#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// #include "lwip/sockets.h"
// #include "lwip/dns.h"
// #include "lwip/netdb.h"
// #include "lwip/igmp.h"

//#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

/*
SPI receiver (slave) example.

This example is supposed to work together with the SPI sender. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. After a transmission has been set up and we're
ready to send/receive data, this code uses a callback to set the handshake pin high. The sender will detect this and start
sending a transaction. As soon as the transaction is done, the line gets set low again.
*/

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#include "nts1_iface.h"

// ----------------------------------------------------

enum {
    k_tx_cmd_event = 0x84U,
    k_tx_cmd_param = 0x85U,
    k_tx_cmd_other = 0x86U,
    k_tx_cmd_dummy = 0x87U
};

enum {
    k_tx_subcmd_other_ack      = 0x3U,
    k_tx_subcmd_other_version  = 0x10U,
    k_tx_subcmd_other_bootmode = 0x11U,
};

enum {
    k_rx_cmd_event = 0x84U,
    k_rx_cmd_param = 0x85U,
    k_rx_cmd_other = 0x86U,
    k_rx_cmd_dummy = 0x87U
};

enum {
    k_rx_subcmd_other_panelid = 0x0U,
    k_rx_subcmd_other_stsreq  = 0x1U,
    k_rx_subcmd_other_ackreq  = 0x3U,
};

typedef struct __nts1_cmd_header {
    uint8_t cmd:3;
    uint8_t panel_id:3;
    uint8_t end_mark:1;
    uint8_t start_bit:1;
} __nts1_cmd_header_t;

// ----------------------------------------------------


#define SPI_MODE 3
#define SPI_BITORDER SPI_SLAVE_BIT_LSBFIRST
#define GPIO_MOSI 23  // G
#define GPIO_HANDSHAKE 16 // R
#define GPIO_MISO 19  // O  
#define GPIO_SCLK 18  // Y
#define GPIO_CS 5     // - None

#ifdef CONFIG_IDF_TARGET_ESP32

#define RCV_HOST    VSPI_HOST
// disable dma, use direct spi buffer
#define DMA_CHAN    0

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#endif

#define BLINK_GPIO 13


#define SPI_TX_BUF_SIZE (32)
#define SPI_TX_BUF_MASK (SPI_TX_BUF_SIZE - 1)

#define SPI_RX_BUF_SIZE (32)
#define SPI_RX_BUF_MASK (SPI_RX_BUF_SIZE - 1)

static uint8_t  s_spi_tx_buf[SPI_TX_BUF_SIZE];
static uint16_t s_spi_tx_ridx;  // Read  Index (from s_spi_tx_buf)
static uint16_t s_spi_tx_widx;  // Write Index (to s_spi_tx_buf)

static uint8_t  s_spi_rx_buf[SPI_RX_BUF_SIZE];
static uint16_t s_spi_rx_ridx;  // Read  Index (from s_spi_rx_buf)
static uint16_t s_spi_rx_widx;  // Write Index (to s_spi_rx_buf)

#define SPI_BUF_INC(idx, bufSize) (((idx+1) == bufSize) ? 0 : idx + 1)

#define SPI_TX_BUF_RESET() (s_spi_tx_ridx = s_spi_tx_widx = 0)
#define SPI_TX_BUF_EMPTY() ((SPI_TX_BUF_MASK & s_spi_tx_ridx) == (SPI_TX_BUF_MASK & s_spi_tx_widx))
#define SPI_RX_BUF_RESET() (s_spi_rx_ridx = s_spi_rx_widx = 0)
#define SPI_RX_BUF_EMPTY() ((SPI_RX_BUF_MASK & s_spi_rx_ridx) == (SPI_RX_BUF_MASK & s_spi_rx_widx))

#define PANEL_ID_MASK    0x38  // Bits 3-5
#define PANEL_CMD_EMARK  0x40  // Bit  6
#define PANEL_START_BIT  0x80  // Bit  7

static uint8_t  s_panel_id = PANEL_ID_MASK; // Bits 3-5 "ppp"="111"
static uint8_t  s_dummy_tx_cmd = (PANEL_ID_MASK + 0xC7); // B'11ppp111;


typedef uint8_t nts1_status_t;

// Configures a GPIO pin for ACK (acknowledge message)
void gpio_init() {
    //Configuration for the handshake line
    gpio_config_t io_conf={
        .mode=GPIO_MODE_OUTPUT,
        .intr_type=GPIO_INTR_DISABLE,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
}



//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
}


esp_err_t spi_init() {
    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=SPI_MODE,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=SPI_BITORDER,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    // Pull down on the Chip Select pin to make it always on
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLDOWN_ONLY);

    //Initialize SPI slave interface
    return spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
}

//Main application
void app_main(void)
{
    int n=0;

    gpio_init();
    esp_err_t ret = spi_init();
    assert(ret==ESP_OK);

    memset(s_spi_rx_buf, 0, SPI_RX_BUF_SIZE);
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t note = 0;
    uint8_t increase = 1;
    while (1) {
        //Clear receive buffer
        memset(s_spi_rx_buf, 0x0, SPI_RX_BUF_SIZE);
        memset(s_spi_tx_buf, 0x0, SPI_TX_BUF_SIZE);

        // send dummy message in the first two iterations
        if (n < 2) {
            s_spi_tx_buf[0] = s_dummy_tx_cmd;
            s_spi_tx_buf[1] = s_dummy_tx_cmd;
            s_spi_tx_buf[2] = s_dummy_tx_cmd;
            s_spi_tx_buf[3] = s_dummy_tx_cmd;
        }

        // every 21 iterations, send a note on message
        if (n%21 == 0) {
            s_spi_tx_buf[0] = 196;
            s_spi_tx_buf[1] = 1;
            s_spi_tx_buf[2] = note;
            s_spi_tx_buf[3] = 10;

            // if we get to the max number, change the direction
            if(note == 127){
               increase = 0;
            }
            if(note == 20) {
                increase = 1;
            }
            // increase or decrease the note number
            note = increase ? note + 1 : note - 1;
        }

        //Set up a transaction of 128 bytes to send/receive
        t.length=SPI_RX_BUF_SIZE*4;
        t.tx_buffer=s_spi_tx_buf;
        t.rx_buffer=s_spi_rx_buf;

        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        uint8_t* cp = s_spi_rx_buf;
        for (uint8_t i = 0; i < SPI_RX_BUF_SIZE; ++cp)
        {
            printf("%02x", *cp);
            i += 1;
        }
        printf("\n\n");

        n++;
    }

}
