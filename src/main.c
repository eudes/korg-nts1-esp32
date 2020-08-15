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

#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "nts1_iface.h"

void psts(char* message, uint8_t sts){
    printf(message);
    printf(";; status: %d\n", sts);
}

void app_main(void)
{
    uint8_t init_status = nts1_init();

    uint8_t note = 0;
    uint8_t increase = 1;
    uint8_t velo = 100;

    while (1)
    {
        //psts("idle", nts1_idle());
        nts1_idle();

        // psts("note on", nts1_note_on(note, velo));
        nts1_note_on(note, velo);

        // if we get to the max number, change the direction
        if (note == 127)
        {
            increase = 0;
        }
        else if (note == 20)
        {
            increase = 1;
        }
        // increase or decrease the note number
        note = increase ? note + 1 : note - 1;
    }
}
