#  Figuring out how to build a custom controller panel for the Korg NTS1
## Or: analyzing a SPI peripheral implementation
### Or: learning microcontroller programming by example

## DISCLAIMER
I am not associated with Korg in any way. This analysis is still a work in progress. The code here is not yet working and it might not even compile. I'm also not an expert in electronics or SoC programming. Anything I say here might be wrong, and if Korg ever publishes an official guide, you'll do better reading that, than this. This is a learning exercise.

## The reference board
Korg provides a Open Hardware reference board, complete with a firmware that communicates with the NTS1.
The board is based on a STM32, and that's the only microcontroller that is supported by the provided libraries.

I don't happen to have a STM32 laying around, and my goal is more towards figuring out the communication protocol and
learning about lower level (C/C++) microcontroller programming, than implementing any particular design for
a controller panel.

So what follows is an investigation on the inner workings of the implementation and, hopefully, another implementation for either the ESP32, the nFR52840, or the ATMega2560, which are the MCUs that I have on hand.

### Links
The NTS1 reference panel lives at:
https://github.com/korginc/nts-1-customizations

The particular variant of the STM32 used for the board is the STM32F030R8T6.
Here's some documentation for the MCU.

Homepage:
https://www.st.com/en/microcontrollers-microprocessors/stm32f030r8.html#resource

Datasheet:
https://www.st.com/resource/en/datasheet/stm32f030r8.pdf

The reference manual:
https://www.st.com/content/ccc/resource/technical/document/reference_manual/c2/f8/8a/f2/18/e6/43/96/DM00031936.pdf/files/DM00031936.pdf/jcr:content/translations/en.DM00031936.pdf

The firmware does extensive use of the HAL peripheral. Here's the docs:
https://www.st.com/resource/en/user_manual/dm00122015-description-of-stm32f0-hal-and-lowlayer-drivers-stmicroelectronics.pdf

## The reference firmware
The reference code is provided in two parts:
- `nts1_iface.c` and `nts1_iface.h`: The STM32-specific implementation (Arduino/variants/NTS1_REF_CP_REVC).
- `nts-1.cpp` and `nts-1.h`: A higher level class for use in Arduino files (Arduino/libraries/NTS-1).

The basic idea of separating the MCU specific code from the higher level abstraction is solid. The problem is that **the whole communication protocol** is implemented in the STM32-specific code. I assume it's done like this because the SMT32 provides native facilities for SPI, including data buffers for managing the read and write processes to the actual GPIOs. However, this makes it a bit tricky to port the protocol, since you essentially have to rebuild the logic that doesn't deal directly with the microcontroller. The higher level class is just a wrapper around the STM32-specific code that doesn't do much. It would have been best to have the higher level class do more of the setup, and the specific implementation just deal with STM32 stuff.

Since we need to reimplement the whole process, the first thing to do is understand what the library is doing under the hood.

### Arduino examples
The code comes with a couple of examples, of which this is the simplest:
```c
#include <nts-1.h>
NTS1 nts1;

void setup() {
  nts1.init();
}
void loop() {
  nts1.idle();
}
```

The `nts1` methods are simple wrappers to the `nts1_iface.c` implementations, so we'll start from there.

### SPI
The first thing you notice when reading the `nts-1.cpp` is that it mentions using `SPI` as a communication protocol. This is nice, since it's a basic full duplex protocol that doesn't have too much boilerplate to deal with.

You have 4 signals:
- CLK (clock):
    - shared. Clock signal. Writers should write on one of the slopes of the clock signal, and readers should sample on the opposite slope. Which slope to use is defined in the SPI Mode. The master ouputs the clock and slaves follow it.
- CS (chip select)/ SS(N) (slave select):
    - one line per slave. The line will normally be HIGH, and the master will make it LOW when it wants to send or receive data for that particular slave.
- MOSI (master out, slave in):
    - shared. The line where the master will send data to the slaves.
- MISO (master in, slave out)
    - shared. The line where the slaves will send data to the master.

SPI Modes are important to take into account, since it dictates which slopes of the clock to use for sampling 
(clock phase, CPHA), and the clock polarity (CPOL). Clock polarity indicates what the base state of the clock is when switching SS low to enable the slave, and the slave starts sampling.
This translates as follows:
- Mode 0: CPOL: 0, CPHA: 0. Clock starts low, data sampled on the first edge, which is the rising edge.
- Mode 1: CPOL: 0, CPHA: 1. Clock start low, data sampled on the second edge, which is the falling edge.
- Mode 2: CPOL: 1, CPHA: 0. Clock starts high, data sampled on the first edge, which is the falling edge.
- Mode 3: CPOL: 1, CPHA: 1. Clock start high, data sampled on the second edge, which is the rising edge.

We also need to establish what the bit order is in each chunk of information (LSB, or MSB).

### Finding the SPI settings
The SPI mode must be known in advance, since SPI doesn't advertise it. The idea is that a master will be able to accomodate different slaves by switching its own settings on the fly when addressing them.

`nts1_iface.c` configures the STM32 as follows:
```c
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
```

This tells us that:
- The panel is working as a **slave**.
- Since it's operating in slave mode, it doesn't need to set a clock speed (it will follow the signal from master).
- The communication is full duplex (`SPI_InitStruct->Direction = SPI_DIRECTION_2LINES`). SPI is full duplex by default, but some chips let you use just half of that if you don't need it.
- It's working with 8 bit packets.
- CPOL is HIGH (1), and CPHA is (1), so our SPI Mode is 3. 
- SS is "soft". This means that it's controlled via software, instead of hardware. There's a mention of "No NSS" in the code, so we can assume that SS is always ON. 
- Bit order is LSB
- TIMode refers to some quirks in the Texas Instrument implementation of SPI, but since it's disabled, I won't worry too much about it yet. 
- CRC is disabled, so no automatic verification is done on the data sent.
- NSSPMode: if enabled, it produces a pulse between each data frame to signal that the device is done reading. It's disabled so I won't worry about it.
- BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2. I'm not quite sure why this is set here. The prescaler is used to divide the system clock by a factor when working as master. As a slave, it shouldn't matter. Might be a default from the IDE.

### STM32 SPI
The STM32 provides hardware support for SPI communication. This means that we don't directly control the GPIO pins to send or receive data. Instead, the STM32 provides C vectors that write to FIFO registers directly, and data is transmitted to the pins from there.

This saves us the hassle of controlling the timing of the messages, since sampling and shifting is taken care of and trasmitted to the FIFO queue for us.

### NTS1 init
We'll analyse the setup process, which should give us a few clues about how the panel works.

This is the method called on setup:
```c
nts1_status_t nts1_init()
{
  // Enables sampling the GPIO pin at clock rate
  // This is mapped to __HAL_RCC_GPIOB_CLK_ENABLE
  ACK_GPIO_CLK_ENABLE();
  
  GPIO_InitTypeDef gpio;

  // Configures a GPIO pin for ACK (acknowledge message)
  /* PANEL ACK */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
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
```

Summing up, the setup method:
- Initializes the GPIO pins as needed for SPI, and for ACK.
- Enables the clock for the different internal devices.
- Resets the buffers and internal variables.

Here, it's also intersting to note that ACK is not part of SPI, so this must be a custom mechanism for the NTS1 to detect the panel and start SPI communications.

### The loop
Once the `setup()` method is finished, our `loop()` method is called repeatedly. This calls `NTS1.idle()` as a first and only step. It won't do much, but there will probably something to learn in it.

```c
nts1_status_t nts1_idle()
{
  // HOST通信の復帰Check
  // This should be true right after executing setup()
  if (s_started) {
    // Checks if the reception buffer is not full
    if (s_spi_chk_rx_buf_space(32)) {
      // Sets the ACK pin to 1
      s_port_startup_ack();
    }
  }
  
  // HOST I/F受信データのIdle処理を優先す
  // as long as the reception buffer is not emptyる
  while (!SPI_RX_BUF_EMPTY()) {
    // Reads from the buffer and executes the handler
    // 受信Bufferにデータあり
    s_rx_msg_handler(s_spi_rx_buf_read());
  }
}
```

We won't get into much detail about what the handler does, but it deals with messages sent from the NTS1 to the peripheral, like NOTE ON messages, param changes, etc.

There's still meat in this bone, with respect to how data is read from the RX buffer, but we should talk first about how data gets into that buffer first.

### Interrupts
As we saw in the setup method, interrupts are enabled for RXNE. They happen whenever the Reception FIFO register is not empty, meaning that an interrupt will be trigger whenever there's data in the input FIFO register.

This is used to load the software reception buffer with data from the FIFO register, and clear the data from the register to allow more data to enter.

The code that handles the interrupt is as follows:
```c
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
    rxdata = s_spi_raw_fifo_pop8(SPI_PERIPH); // DR読み出しでRXNE
    // And write it into the software RX buffer
    // This function only writes if the buffer has enough space to accomodate the byte
    if (!s_spi_rx_buf_write(rxdata)) {
      // If there's not enough space in the buffer
      // this reset the index read and write indexes for the buffer
      // which will cause it to start writing into the buffer from the beginning
      SPI_RX_BUF_RESET();
    } 
    else {
      // If there is enough space
      if (!s_spi_chk_rx_buf_space(32)) {
         // and the buffer cannot accomodate 32 bits more
         // the ACK pin is set to 0
         s_port_wait_ack();
         // I assume this means the NTS1 will stop sending data
      } else { //
         // otherwise the ACK pin is set to 1
         s_port_startup_ack();
         // which will allow the NTS1 to send data
      }
    }
  }

  // HOST <- PANEL
  if (!SPI_TX_BUF_EMPTY()) { // 送信Bufferにデータあり
    // If there's data to be sent
    txdata = s_spi_tx_buf_read();
    if (txdata & 0x80) { // Statusの時は、EndMark
      // and the data contains 0x80
      if (!SPI_TX_BUF_EMPTY()) { // 送信Buffer
        // an EMARK is set on the data
        txdata |= PANEL_CMD_EMARK;
        // Note: this will set endmark on almost any status, especially those who have pending data,
        //       which seems to contradict the endmark common usage of marking only the last command of a group
      }
    }
    // Data is sent to the Tx FIFO register
    s_spi_raw_fifo_push8(SPI_PERIPH, txdata);
  }
  else { // 
    // Dummy data is sent to the TX FIFO register
    s_spi_raw_fifo_push8(SPI_PERIPH, s_dummy_tx_cmd);
  }
}
```

It's interesting to note that the interrupt handler does both the reading and the writing to the Rx and Tx registers. s_spi_raw_fifo_push8() is only called from here (and the setup method). This means that the panel only sends data when there's data in the input FIFO. The public "send/write" methods for the NTS1 class only write to the software Tx buffer.

### Writing to the SPI FIFO
The following are the only two places where direct access to the FIFO registers happens. This code is interesting because of the way the memory address for the SPI_DR register is calculated and derefenced.

```c
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
```
The FIFO registers are 32 bits. A buffer overrun event is triggered if data is written to them when full.
An interrupt is triggered when the FIFOs are ready to be used:
- Rx FIFO: the RXNE (not empty) interrupt is triggered whenever a frame has entered the FIFO, meaning that data should be read out of it.
- Tx FIFO: the TXE (empty) interrupt is triggered when the TXFIFO level is less than or equal to half of its capacity, meaning that data can be sent to it.

Both RXNE and TXE are flags that can be polled, or be set to trigger an interrupt, as the code here is doing for RXNE.

## A testing application
With the information gathered, we can begin building an application in Arduino that will allow us to connect to the NTS1 controller, set our ACK pin to 1, and start receiving data.


