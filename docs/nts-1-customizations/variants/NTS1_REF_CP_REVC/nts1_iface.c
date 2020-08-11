/** 
 * @file nts1_iface.c
 * @brief C API handling communication with the NTS-1 digital kit's main board.
 *
 * Provides initialization of necessary peripherals, handling of low level 
 * protocol details and convenience functions for bidrectional communication
 * with the NTS-1 digital kit's main board via SPI.
 *   
 * BSD 3-Clause License
 *  Copyright (c) 2020, KORG INC.
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //*/

#include "nts1_iface.h"

#include <assert.h>

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_spi.h"

#include "stm32_def.h"
#include "PeripheralPins.h"
#include "PinAF_STM32F1.h"
#include "pinconfig.h"
#include "spi_com.h"

#define SPI_PERIPH       SPI2
#define SPI_MISO_PORT    GPIOB
#define SPI_MISO_PIN     GPIO_PIN_14
#define SPI_MOSI_PORT    GPIOB
#define SPI_MOSI_PIN     GPIO_PIN_15
#define SPI_SCK_PORT     GPIOB
#define SPI_SCK_PIN      GPIO_PIN_13
#define SPI_GPIO_AF      GPIO_AF0_SPI2

#define SPI_IRQn         SPI2_IRQn
#define SPI_IRQ_PRIORITY 1
#define SPI_IRQ_HANDLER  SPI2_IRQHandler

#define SPI_GPIO_CLK_ENA()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_FORCE_RESET()    __HAL_RCC_SPI2_FORCE_RESET()
#define SPI_RELEASE_RESET()  __HAL_RCC_SPI2_RELEASE_RESET()
#define SPI_CLK_ENABLE()     __HAL_RCC_SPI2_CLK_ENABLE()
#define SPI_CLK_DISABLE()    __HAL_RCC_SPI2_CLK_DISABLE()

#define ACK_PORT         GPIOB
#define ACK_PIN          GPIO_PIN_12

#define ACK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define PANEL_ID_MASK    0x38  // Bits 3-5 // 00111000 
#define PANEL_CMD_EMARK  0x40  // Bit  6   // 01000000
#define PANEL_START_BIT  0x80  // Bit  7   // 10000000

#define SPI_TX_BUF_SIZE (0x200)
#define SPI_TX_BUF_MASK (SPI_TX_BUF_SIZE - 1)

#define SPI_RX_BUF_SIZE (0x200 )
#define SPI_RX_BUF_MASK (SPI_RX_BUF_SIZE - 1)

#ifndef true 
#define true 1
#endif

#ifndef false
#define false 0
#endif

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

static SPI_HandleTypeDef s_spi;

static uint8_t  s_panel_id = PANEL_ID_MASK; // Bits 3-5 "ppp"="111"
static uint8_t  s_dummy_tx_cmd = (PANEL_ID_MASK + 0xC7); // B'11ppp111;

static uint8_t  s_started;

static uint8_t  s_spi_tx_buf[SPI_TX_BUF_SIZE];
static uint16_t s_spi_tx_ridx;  // Read  Index (from s_spi_tx_buf)
static uint16_t s_spi_tx_widx;  // Write Index (to s_spi_tx_buf)

static uint8_t  s_spi_rx_buf[SPI_RX_BUF_SIZE];
static uint16_t s_spi_rx_ridx;  // Read  Index (from s_spi_rx_buf)
static uint16_t s_spi_rx_widx;  // Write Index (to s_spi_rx_buf)

static uint8_t  s_panel_rx_status;
static uint8_t  s_panel_rx_data_cnt;
static uint8_t  s_panel_rx_data[127];

// ----------------------------------------------------

#define SPI_TX_BUF_RESET() (s_spi_tx_ridx = s_spi_tx_widx = 0)
#define SPI_TX_BUF_EMPTY() ((SPI_TX_BUF_MASK & s_spi_tx_ridx) == (SPI_TX_BUF_MASK & s_spi_tx_widx))
#define SPI_RX_BUF_RESET() (s_spi_rx_ridx = s_spi_rx_widx = 0)
#define SPI_RX_BUF_EMPTY() ((SPI_RX_BUF_MASK & s_spi_rx_ridx) == (SPI_RX_BUF_MASK & s_spi_rx_widx))

#define SPI_BUF_INC(idx, bufSize) (((idx+1) == bufSize) ? 0 : idx + 1)

// ----------------------------------------------------

static inline void s_port_startup_ack(void)
{
  // This sets the ACK_PIN GPIO pin to 1
  // ACK_PORT is the GPIO device (of which the STM32 has 2)
  // BSRR is the Bit Set Register
  // Setting this register will write to the GPIO pin
  // This is an atomic operation (takes 1 system clock tick),
  // as oppossed to calling a method which would take more time
  ACK_PORT->BSRR = ACK_PIN;
}

static inline void s_port_wait_ack(void)
{
  // This sets the ACK_PIN GPIO pin to 0
  // Same as above, but the register is the Bit Reset Register
  ACK_PORT->BRR = ACK_PIN;
}

// ----------------------------------------------------

static inline void s_spi_raw_fifo_push8(SPI_TypeDef* SPIx, uint8_t data)
{
  // SPIx is a pointer, so will a contain a memory address of the SPI device
  // 0x0C is the offset of the Data Register with respect to the SPI device
  const uint32_t spix_dr = (uint32_t)SPIx + 0x0C;
  // spix_dr is the address of the SPIX_DR data register
  // Reading from this address will return the oldest frame of data from the Rx FIFO
  // Writing to this address will write to the end of the Tx FIFO

  // In this case, we are writing to it
  *(__IO uint8_t *) spix_dr = data;
}

static inline uint8_t s_spi_raw_fifo_pop8(SPI_TypeDef* SPIx)
{
  const uint32_t spix_dr = (uint32_t)SPIx + 0x0C;   

  // In this case, we are reading it
  return *(__IO uint8_t *) spix_dr;
}

static uint8_t s_spi_chk_rx_buf_space(uint16_t size)
{
  uint16_t count;

  if (s_spi_rx_ridx <= s_spi_rx_widx) {
    count = (SPI_RX_BUF_SIZE + s_spi_rx_ridx) - s_spi_rx_widx;
  } else {
    count = s_spi_rx_ridx - s_spi_rx_widx;
  }
  return (count > size);
}

static uint8_t s_spi_rx_buf_write(uint8_t data) 
{
  uint16_t bufdatacount;
  if (s_spi_rx_ridx <= s_spi_rx_widx) {
    bufdatacount = s_spi_rx_widx - s_spi_rx_ridx;
  } else {
    bufdatacount = SPI_RX_BUF_SIZE + s_spi_rx_widx - s_spi_rx_ridx;
  }
  if (bufdatacount < (SPI_RX_BUF_SIZE - 2)) {
    s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_widx] = data;
    s_spi_rx_widx = SPI_BUF_INC(s_spi_rx_widx, SPI_RX_BUF_SIZE);
    return true;
  }
  return false;
}

static uint8_t s_spi_rx_buf_read(void)
{
  const uint8_t data = s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_ridx];
  s_spi_rx_ridx = SPI_BUF_INC(s_spi_rx_ridx, SPI_RX_BUF_SIZE);
  return data;
}

static uint8_t s_spi_chk_tx_buf_space(uint16_t size)
{
  uint16_t count;
  if (s_spi_tx_ridx <= s_spi_tx_widx) {
    count = SPI_TX_BUF_SIZE + s_spi_tx_ridx - s_spi_tx_widx;
  } else {
    count = s_spi_tx_ridx - s_spi_tx_widx;
  }
  return (count > size);
}

// writes the byte to the buffer at the appropiate position
// and moves the buffer pointers
static void s_spi_tx_buf_write(uint8_t data)
{
  s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_widx] = data;
  s_spi_tx_widx = SPI_BUF_INC(s_spi_tx_widx, SPI_TX_BUF_SIZE);
}

static uint8_t s_spi_tx_buf_read(void)
{
  const uint8_t data = s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_ridx];
  s_spi_tx_ridx = SPI_BUF_INC(s_spi_tx_ridx, SPI_TX_BUF_SIZE);
  return data;
}

// ----------------------------------------------------

static inline void s_spi_struct_init(SPI_InitTypeDef* SPI_InitStruct)
{
  SPI_InitStruct->Mode = SPI_MODE_SLAVE;
  SPI_InitStruct->Direction = SPI_DIRECTION_2LINES;
  SPI_InitStruct->DataSize = SPI_DATASIZE_8BIT;
  SPI_InitStruct->CLKPolarity = SPI_POLARITY_HIGH;
  SPI_InitStruct->CLKPhase = SPI_PHASE_2EDGE;
  SPI_InitStruct->NSS = SPI_NSS_SOFT;
  SPI_InitStruct->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SPI_InitStruct->FirstBit = SPI_FIRSTBIT_LSB;
  SPI_InitStruct->TIMode = SPI_TIMODE_DISABLE;
  SPI_InitStruct->CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct->CRCPolynomial = 7;
  SPI_InitStruct->CRCLength = SPI_CRC_LENGTH_DATASIZE;
  SPI_InitStruct->NSSPMode = SPI_NSS_PULSE_DISABLE;
}

static inline void s_spi_enable_pins()
{
  // Same as ACK_GPIO_CLK_ENABLE,
  // mapped to __HAL_RCC_GPIOB_CLK_ENABLE
  SPI_GPIO_CLK_ENA();
  
  GPIO_InitTypeDef gpio;
  
  /* Enable SCK, MOSI, MISO. No NSS. */
  /* Peripherals alternate function */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pull = GPIO_NOPULL;
  // GPIO pins set to alternate function, which allows alternating the pins
  // between GPIO functions and internal peripheral functions
  // This is a requirement of the SPI device in the STM32
  // See Reference Manual, Figure 274
  gpio.Alternate = SPI_GPIO_AF;
  
  gpio.Pin = SPI_MISO_PIN;
  HAL_GPIO_Init(SPI_MISO_PORT, &gpio);
  
  gpio.Pin = SPI_MOSI_PIN;
  HAL_GPIO_Init(SPI_MOSI_PORT, &gpio);
  
  gpio.Pin = SPI_SCK_PIN;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI_SCK_PORT, &gpio);
  
}

HAL_StatusTypeDef s_spi_init()
{
  // Enables the clock for the SYSCFG register
  // which is used for specific configurations of memory and 
  // DMA requests remap and to control special I/O features.
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  // Configures the GPIO pins for SPI
  // See below
  s_spi_enable_pins();

  // Resets and enables clock signals for SPI
  SPI_FORCE_RESET();
  SPI_RELEASE_RESET();
  SPI_CLK_ENABLE();
  
  // Configures SPI as seen above
  // Note that SPI_PERIPH is mapped to SPI2, which is one
  // of the 2 available SPI devices in the MCU
  s_spi.Instance = SPI_PERIPH;
  s_spi_struct_init(&(s_spi.Init));
  
  // Initializes the SPI driver
  const HAL_StatusTypeDef res = HAL_SPI_Init(&s_spi);
  if (res != HAL_OK) {
    return res;
  }

  // Sets the interrupt handler in the NVIC (Nested Vector Interrupt Controller)
  HAL_NVIC_SetPriority(SPI_IRQn, SPI_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(SPI_IRQn);
  
  // Adds (bitwise OR) RXNE to the SPI control register 2
  // This enables the "RX buffer not empty" interrupt
  // See Reference Manual 28.5.10
  SPI_PERIPH->CR2 |= SPI_IT_RXNE;

  // Empties the buffers for transmission and reception
  // and reset counters
  s_panel_rx_status = 0;
  s_panel_rx_data_cnt = 0;
  SPI_RX_BUF_RESET();
  SPI_TX_BUF_RESET();
  
  // Finally enables HAL SPI
  __HAL_SPI_ENABLE(&s_spi);

  return HAL_OK;
}


HAL_StatusTypeDef s_spi_teardown()
{
  __HAL_SPI_DISABLE(&s_spi);
  SPI_CLK_DISABLE();
  return HAL_OK;
}

// ----------------------------------------------------

static uint8_t s_tx_cmd_event(const nts1_tx_event_t *event, uint8_t endmark) 
{
  assert(event != NULL);
  if (!s_spi_chk_tx_buf_space(4)) 
     return false;
  // if (s_panel_id & PANEL_ID_MASK) + (endmark) > 0, add PANEL_CMD_EMARK bit to the 
  // command byte
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_event | PANEL_CMD_EMARK) : k_tx_cmd_event; 
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(event->event_id & 0x7F);
  s_spi_tx_buf_write(event->msb & 0x7F);
  s_spi_tx_buf_write(event->lsb & 0x7F);
  return true;
}

static uint8_t s_tx_cmd_param_change(const nts1_tx_param_change_t *param_change, uint8_t endmark) 
{
  assert(param_change != NULL);
  if (!s_spi_chk_tx_buf_space(4)) 
     return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_param | PANEL_CMD_EMARK) : k_tx_cmd_param; 
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(param_change->param_id & 0x7F);
  s_spi_tx_buf_write(param_change->param_subid & 0x7F);
  s_spi_tx_buf_write(param_change->msb & 0x7F);
  s_spi_tx_buf_write(param_change->lsb & 0x7F);
  return true;
}

static uint8_t s_tx_cmd_other_ack(uint8_t endmark) 
{
  if (!s_spi_chk_tx_buf_space(3)) 
     return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other; 
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(3);
  s_spi_tx_buf_write(k_tx_subcmd_other_ack);
  return true;
}

static uint8_t s_tx_cmd_other_version(uint8_t endmark) 
{
  if (!s_spi_chk_tx_buf_space(5)) 
     return false; 
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other; 
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(5);
  s_spi_tx_buf_write(k_tx_subcmd_other_version);
  s_spi_tx_buf_write(1);
  s_spi_tx_buf_write(0);
  return true;
}

static uint8_t s_tx_cmd_other_bootmode(uint8_t endmark) 
{
  if (!s_spi_chk_tx_buf_space(4))
    return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other; 
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(4);
  s_spi_tx_buf_write(k_tx_subcmd_other_bootmode);
  s_spi_tx_buf_write(0);
  return true;
}

// ----------------------------------------------------

static uint8_t s_dummy_buffer[64];
#define RX_EVENT_MAX_DECODE_SIZE 64
static uint8_t s_rx_event_decode_buf[RX_EVENT_MAX_DECODE_SIZE] = {0};

// This is the handler itself
static void s_rx_msg_handler(uint8_t data)
{
  // data is 1 byte of the input buffer
  // If data byte starts with 10000000
  if (data >= PANEL_START_BIT) {
    // Status byte
    // resets the counter
    s_panel_rx_data_cnt = 0;
    // data = data AND not(PANEL_CMD_EMARK)
    // PANEL_CMD_EMARK is 01000000
    // not(PANEL_CMD_EMARK) is 10111111
    // so, the following discards the bit at position 6 (starting from pos 0)
    data &= ~PANEL_CMD_EMARK;

    if (data == 0xBEU) { // 10111110:Panel ID allocation
      // if data == 1x111110
      s_panel_rx_status = data & ~PANEL_ID_MASK; // discards bits 3,4 and 5
      //s_panel_rx_status  = 10111110 * 11000111 => 10000110
    } else if (
      (data & PANEL_ID_MASK) // bits 3 4 5 of data (00xxx000)
      == 
      (s_panel_id & PANEL_ID_MASK) // 00111000
      ) { // if data contains (xx111xxx)
      s_panel_rx_status = data & ~PANEL_ID_MASK; // produces 10000xxx
    } else {
      s_panel_rx_status = 0;  // cancel any previous command reception
    }
    // exits the method to parse next byte
    return;
  }
  // the previous section stores in s_panel_rx_status the first byte
  // received, identified by having its first bit set to 1 (1xxxxxxx)
  // and it processes it to discard irrelevant data

  // Relevant statuses:
  // if data == 1x111110 => active_cmd = 10000110
  // if data == 1x111xxx => active_cmd = 10000xxx
  // otherwise, status byte is discarded
  
  // Stored status byte
  const uint8_t active_cmd = s_panel_rx_status;
  
  // Now we process bytes that don't start with 1
  switch (active_cmd) {
    // 1x111 100
  case k_rx_cmd_event:
    { 
      // this will push stuff into the array
      // for future use
      s_panel_rx_data[s_panel_rx_data_cnt++] = data;
      
      // transactions are at least 3 bytes long
      if (s_panel_rx_data_cnt < 2) 
        break; // need more data
      
      // The second byte always contains the length of the message
      // so this will wait until all the bytes have been received to process the message
      // It also subtracts 1 from the indicated size, because the first byte
      // (the status byte), is not stored in this array
      if (s_panel_rx_data_cnt < (s_panel_rx_data[0] - 1)) 
        break; // need more data
      
      /*++++++++++++++++++++++++++++++++++++++++++++++
        CMD4 : Event
        1st    :[1][0][ppp][100]
        2nd    :[0][sssssss] Size
        3rd    :[0][eeeeeee] Event ID
        4th    :[0][ddddddd] Data word
        ...
        +++++++++++++++++++++++++++++++++++++++++++++*/

      
      const nts1_rx_event_header_t *rx_event = (const nts1_rx_event_header_t *)s_panel_rx_data;
      // payload is pointer to the position of s_panel_rx_data + nts1_rx_event_header_t
      // which I guess is the memory position of the actual data
      const uint8_t *payload = s_panel_rx_data + sizeof(nts1_rx_event_header_t);

      const uint32_t payload_size7 = (rx_event->size - sizeof(nts1_rx_event_header_t) - 1);
      const uint32_t payload_size8 = nts1_size_7to8(payload_size7);
      if (payload_size8 > RX_EVENT_MAX_DECODE_SIZE) {
        // Reset rx status
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
      }
      nts1_convert_7to8(s_rx_event_decode_buf, payload, payload_size7);
      
      switch (rx_event->event_id) {
      case k_nts1_rx_event_id_note_off:
        if (payload_size8 == sizeof(nts1_rx_note_off_t))
          nts1_handle_note_off_event((const nts1_rx_note_off_t *)s_rx_event_decode_buf);
        break;
      case k_nts1_rx_event_id_note_on:
        if (payload_size8 == sizeof(nts1_rx_note_on_t))
          nts1_handle_note_on_event((const nts1_rx_note_on_t *)s_rx_event_decode_buf);
        break;
      case k_nts1_rx_event_id_step_tick:
        nts1_handle_step_tick_event();
        break;
      case k_nts1_rx_event_id_unit_desc:
        //if (payload_size8 == sizeof(nts1_rx_unit_desc_t))
          nts1_handle_unit_desc_event((const nts1_rx_unit_desc_t *)s_rx_event_decode_buf);
        break;
      case k_nts1_rx_event_id_edit_param_desc:
        if (payload_size8 == sizeof(nts1_rx_edit_param_desc_t)) {
          nts1_handle_edit_param_desc_event((const nts1_rx_edit_param_desc_t *)s_rx_event_decode_buf);
        }
        break;
      case k_nts1_rx_event_id_value:
        if (payload_size8 == sizeof(nts1_rx_value_t))
          nts1_handle_value_event((const nts1_rx_value_t *)s_rx_event_decode_buf);
        break;
      default:
        break;
      }
      
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
    }
    break;

  // 1x111 101
  case k_rx_cmd_param:
    {
      s_panel_rx_data[s_panel_rx_data_cnt++] = data;

      if (s_panel_rx_data_cnt < 4)
        break; // need more data

      /*++++++++++++++++++++++++++++++++++++++++++++++
        CMD5 : Param Change
        1st    :[1][0][ppp][101]
        2nd    :[0][eeeeeee] Param ID
        3rd    :[0][sssssss] Param Sub ID
        4th    :[0][hhhhhhh] MSB
        5th    :[0][lllllll] LSB
        +++++++++++++++++++++++++++++++++++++++++++++*/

      const nts1_rx_param_change_t *rx_param = (const nts1_rx_param_change_t *)s_panel_rx_data;
      nts1_handle_param_change(rx_param);
      
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
    }
    break;
    
  // 1x111 110
  case k_rx_cmd_other:
    {
      // continue reading until we have the full command
      s_panel_rx_data[s_panel_rx_data_cnt++] = data;
      // it's important to note that here, if s_panel_rx_data[0] (size)
      // is 0, the break will never happen
      if (s_panel_rx_data_cnt < (s_panel_rx_data[0] - 1)) 
        break; // need more data

      if (s_panel_rx_data_cnt < 2) {
        // Command too short - ignore and reset
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
      }
      
      // switches on the 3rd byte (MessageID)
      switch (s_panel_rx_data[1]) {
      case k_rx_subcmd_other_panelid: // Panel ID specified ("ppp" is left here but not here)
        /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-0 :PanelID designation.
          Only in this case, specify ppp=7.
          1st    :[1][0][111][110] ppp=7 use
          2nd    :[0][0000100] Size=4
          3rd    :[0][0000000] MessageID = 0
          4th    :[0][0000PPP] The PanelID number you specify.
          +++++++++++++++++++++++++++++++++++++++++++++*/
        if (s_panel_rx_data_cnt >= 3 && s_panel_rx_data[0] == 4) {
          s_panel_id = ((s_panel_rx_data[2] & 0x07) << 3) & PANEL_ID_MASK;
          s_dummy_tx_cmd = s_panel_id | 0xC7; // B'11ppp111;
          // Send Version to HOST
          s_tx_cmd_other_version(false);
          // Send All SW Pattern to HOST
          s_tx_cmd_other_bootmode(true);
        }
        // Reset rx status
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
        
      case k_rx_subcmd_other_stsreq:
        /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-1 :Status Request
          Request to send the current SwitchPattern (CMD6-17) and all knob commands.
          1st    :[1][0][ppp][110]
          2nd    :[0][0000011] Size=3
          3rd    :[0][0000001] MessageID = 1
          +++++++++++++++++++++++++++++++++++++++++++++*/
        s_tx_cmd_other_bootmode(true);
        // Reset rx status
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
        
      case k_rx_subcmd_other_ackreq: // Panel ACK req
        /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-3 :ACK request
          For checking whether the Panel is operating normally.
          When Panel receives this, it returns an ACK command.
          1st    :[1][0][ppp][110]
          2nd    :[0][0000011] Size=3
          3rd    :[0][0000011] MessageID = 3
          +++++++++++++++++++++++++++++++++++++++++++++*/
        s_tx_cmd_other_ack(true);
        // Reset rx status
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
        
      default:
        // Undefined command - ignore and reset
        s_panel_rx_status = 0;
        s_panel_rx_data_cnt = 0;
        break;
      }
    } // end case k_rx_cmd_other:
  // 1x111 111
  case k_rx_cmd_dummy:
    // continues to default
  default:
    // resets
    s_panel_rx_status = 0;    // Clear save status
    s_panel_rx_data_cnt = 0; // Initialize data count
    break;
  }
}

// ----------------------------------------------------

extern void SPI_IRQ_HANDLER()
{  
  volatile uint16_t sr;
  uint8_t txdata, rxdata;
  
  // HOST -> PANEL
  // While the RX FIFO is not empty 
  // This is signified by the RXNE flag in the SPI Status Registry being 1
  // and calculated from the SPI Status registry
  // by doing a bitwise AND between the SPI_SR_RXNE flag and the SR itself
  // see Reference Manual 28.9.3
  while ((sr = SPI_PERIPH->SR) & SPI_SR_RXNE) {
    // Take out 8 bits from the FIFO
    rxdata = s_spi_raw_fifo_pop8(SPI_PERIPH); // DR read clears RXNE flag
    // And write it into the software RX buffer
    // This function only writes if the buffer has enough space to accomodate the byte
    if (!s_spi_rx_buf_write(rxdata)) { // write to the RX buffer
      // If there's not enough space in the buffer
      // this resets the index read and write indexes for the buffer
      // which will cause it to start writing into the buffer from the beginning
      
      // If RxBuf is full, reset it.
      SPI_RX_BUF_RESET();
    } 
    else {
      // If there is not 32 bits of space
      if (!s_spi_chk_rx_buf_space(32)) {
         // the ACK pin is set to 0
         s_port_wait_ack();
         // I assume this means the NTS1 will stop sending data
      } else { //Remaining buffer
         // otherwise the ACK pin is set to 1
         s_port_startup_ack();
         // which will allow the NTS1 to send data
      }
    } 
  }
  // so, basically, as long as there is data in the RX FIFO
  // read it and put it in the RX buffer
  // if there's no space, the RX write and read position pointers
  // are reset, so that we can start writing from the beginning of the
  // buffer again.
  // Do this until there's no more data in the RC FIFO (the RXNE flag clears then)

  // HOST <- PANEL
  if (!SPI_TX_BUF_EMPTY()) { // Send buffer has data
    // If there's data to be sent
    txdata = s_spi_tx_buf_read(); // read it and advance the read idx pointer
    if (txdata & PANEL_START_BIT) { // In Status, check whether EndMark is added.
      // and the data contains 0x80 (is a Status message)
      if (!SPI_TX_BUF_EMPTY()) { // There is (more) data (after the status) to be sent next in the send buffer
        // an EMARK is set on the status
        txdata |= PANEL_CMD_EMARK;
        // Note: this will set endmark on almost any status, especially those who have pending data,
        //       which seems to contradict the endmark common usage of marking only the last command of a group
      }
    }
    // Data is sent to the Tx FIFO register
    s_spi_raw_fifo_push8(SPI_PERIPH, txdata);
  }
  else { // Set the dummy because the send buffer is empty.
    // Dummy data is sent to the TX FIFO register
    s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  }

  // write one byte of data from the Tx buffer to the Tx FIFO.
  // if the byte is a Status message, and it has more data after it,
  // add the ENDMARK (byte 7 set to 1)
  // If there's no data to be sent, write the DUMMY message
}
// ----------------------------------------------------
  
nts1_status_t nts1_init()
{
  // Enables sampling the GPIO pin at clock rate
  // This is mapped to __HAL_RCC_GPIOB_CLK_ENABLE
  ACK_GPIO_CLK_ENABLE();
  
  GPIO_InitTypeDef gpio;

  // Configures a GPIO pin for ACK (acknowledge message)
  /* PANEL ACK */
  gpio.Mode = GPIO_MODE_OUTPUT_PP; // normal output, not Open Collector or Open Drain
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = 0;
  gpio.Pin = ACK_PIN;
  HAL_GPIO_Init(ACK_PORT, &gpio);
  
  // More on this below
  HAL_StatusTypeDef res = s_spi_init();
  if (res != HAL_OK) 
    return (nts1_status_t)res;
  
  // Fills the hardware transmission FIFO register with dummy data
  // Fill TX FIFO
  s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  //*/
  
  // Sets the ACK pin to 1
  // More below
  s_port_startup_ack();
  s_started = true;
  
  return k_nts1_status_ok;
}

nts1_status_t nts1_teardown()
{
  HAL_StatusTypeDef res = s_spi_teardown();

  return (nts1_status_t)res;
}

nts1_status_t nts1_idle()
{
  // Return of HOST communication Check
  // This should be true right after executing setup()
  if (s_started) {
    // Checks if the reception buffer is not full
    if (s_spi_chk_rx_buf_space(32)) {
      // Sets the ACK pin to 1
      s_port_startup_ack();
    }
  }
  
  // HOST I/F Give priority to Idle processing of received data
  // As long as the reception buffer is not empty
  while (!SPI_RX_BUF_EMPTY()) {
    // Reads from the buffer and executes the handler
    // Data in receive buffer
    s_rx_msg_handler(s_spi_rx_buf_read());
  }
}

// ----------------------------------------------------
  
nts1_status_t nts1_send_events(nts1_tx_event_t *events, uint8_t count)
{
  assert(events != NULL);
  for (uint8_t i=0; i < count; ++i) {
    if (!s_tx_cmd_event(&events[i], (i == count-1))) {
      return k_nts1_status_busy;
    }
  }
  return k_nts1_status_ok;
}

nts1_status_t nts1_send_param_changes(nts1_tx_param_change_t *param_changes, uint8_t count)
{
  assert(param_changes != NULL);
  for (uint8_t i=0; i < count; ++i) {
    if (!s_tx_cmd_param_change(&param_changes[i], (i == count-1))) {
      return k_nts1_status_busy;
    }
  }
  return k_nts1_status_ok;
}

uint32_t nts1_convert_7to8(uint8_t *dest8, const uint8_t *src7, uint32_t size7) {
  const uint32_t size8 = nts1_size_7to8(size7);
  const uint8_t *src7_e = src7 + size7;
  for (uint32_t i7 = 0, i8 = 0; i7 < size7; ++i7) {
    const uint8_t i7mod8 = i7 % 8;
    switch (i7mod8) {
    case 0:
      dest8[i8++] = src7[i7] & 0x7F;
      break;
    case 7:
      dest8[i8-1] |= (src7[i7] & 0x7F)<<1;
      break;
    default:
      {
        const uint8_t offset = 8-i7mod8;
        const uint8_t src = src7[i7];
        dest8[i8-1] |= (src & ((1U<<i7mod8)-1))<<offset;
        dest8[i8++] = (src & 0x7F)>>i7mod8;
      }
      break;
    }
  }
  return size8;
}

uint32_t nts1_convert_8to7(uint8_t *dest7, const uint8_t *src8, uint32_t size8) {
  const uint32_t size7 = nts1_size_8to7(size8);
  const uint8_t * dest7_e = dest7 + size7;
  for (uint32_t i7 = 0, i8 = 0; i7 < size7; ++i7) {
    const uint8_t i7mod8 = i7 % 8;
    switch (i7mod8) {
    case 0:
      dest7[i7] = src8[i8++] & 0x7F;
      break;
    case 7:
      dest7[i7] = (src8[i8-1] & (0x7F<<1))>>1;
      break;
    default:
      {
        const uint8_t offset = 8-i7mod8;
        uint8_t dest = (src8[i8-1] & (0xFFU<<offset))>>offset;
        dest |= (src8[i8++] & (0x7F>>i7mod8))<<i7mod8;
        dest7[i7] = dest;
      }
      break;
    }
  }
  return size7;
}

nts1_status_t nts1_param_change(uint8_t id, uint8_t subid, uint16_t value) {
  nts1_tx_param_change_t param;
  param.param_id = id;
  param.param_subid = subid;
  param.msb = (value >> 7) & 0x7F;
  param.lsb = value & 0x7F;
  return nts1_send_param_change(&param);
}

nts1_status_t nts1_note_on(uint8_t note, uint8_t velo) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_note_on;
  event.msb = note & 0x7F;
  event.lsb = velo & 0x7F;
  return nts1_send_event(&event); 
}

nts1_status_t nts1_note_off(uint8_t note) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_note_off;
  event.msb = note & 0x7F;
  event.lsb = 0x00;
  return nts1_send_event(&event);  
}

nts1_status_t nts1_req_sys_version(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_value;
  event.msb = k_param_id_sys_version;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_param_value(uint8_t id, uint8_t subid) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_value;
  event.msb = id;
  event.lsb = subid;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_osc_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_osc_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_osc_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_osc_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_osc_edit_param_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_edit_param_desc;
  event.msb = k_param_id_osc_type;
  event.lsb = idx;
  return nts1_send_event(&event);  
}

nts1_status_t nts1_req_filt_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_filt_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_filt_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_filt_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_ampeg_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_ampeg_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_ampeg_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_ampeg_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_mod_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_mod_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_mod_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_mod_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_del_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_del_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_del_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_del_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_rev_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_rev_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_rev_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_rev_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_arp_pattern_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_arp_pattern;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_pattern_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_arp_pattern;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}

nts1_status_t nts1_req_arp_intervals_count(void) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_arp_intervals;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_intervals_desc(uint8_t idx) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_arp_intervals;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);    
}


// ----------------------------------------------------
