#  Figuring out how to build a custom controller panel for the Korg NTS1
## Or: analyzing a SPI peripheral implementation
### Or: learning microcontroller programming by example

## DISCLAIMER
I am not associated with Korg in any way. This analysis is still a work in progress. The code here might not work for you. I'm also not an expert in electronics or SoC programming. Anything I say here might be wrong, and if Korg ever publishes an official guide, you'll do better reading that, than this. This is a learning exercise.

## The reference board
Korg provides an Open Hardware reference board, complete with a firmware that communicates with the NTS1.
The board is based on a STM32, and that's the only microcontroller that is supported by the provided libraries.

I don't happen to have a STM32 laying around, and my goal is more towards figuring out the communication protocol and learning about lower level (C/C++) microcontroller programming, than implementing any particular design for
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

## This repo
- docs/nts-1-customizations : the original [`nts-1-customizations` repo](https://github.com/korginc/nts-1-customizations) code, with added comments and japanese sentences translated.
- docs/hex : contains a python utility to translate hex messages into binary (0s and 1s).
- examples/ : working examples for the topics I talk about in this page. Snippets are taken from there.
- src/ : my latest progress, the code that gets uploaded to my ESP32. Code that works is taken from here and put into `examples/.

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
    rxdata = s_spi_raw_fifo_pop8(SPI_PERIPH); // DR read clears RXNE flag
    // And write it into the software RX buffer
    // This function only writes if the buffer has enough space to accomodate the byte
    if (!s_spi_rx_buf_write(rxdata)) {
      // If there's not enough space in the buffer
      // this resets the index read and write indexes for the buffer
      // which will cause it to start writing into the buffer from the beginning
      
      // If RxBuf is full, reset it.
      SPI_RX_BUF_RESET();
    } 
    else {
      // If there is enough space
      if (!s_spi_chk_rx_buf_space(32)) {
         // and the buffer cannot accomodate 32 bits more
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
  // now the rx FIFO is empty (the RXNE flag has cleared)

  // HOST <- PANEL
  if (!SPI_TX_BUF_EMPTY()) { // Send buffer has data
    // If there's data to be sent
    txdata = s_spi_tx_buf_read();
    if (txdata & 0x80) { // In Status, check whether EndMark is added.
      // and the data contains 0x80
      if (!SPI_TX_BUF_EMPTY()) { // There is data to be sent next in the send buffer
        // an EMARK is set on the data
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
}
```

It's interesting to note that the interrupt handler does both the reading and the writing to the Rx and Tx registers. s_spi_raw_fifo_push8() is only called from here (and the setup method). This means that the panel only sends data when there's data in the input FIFO. The public "send/write" methods for the NTS1 class only write to the software Tx buffer.

### Writing to the SPI FIFO
The following are the only two places where direct access to the FIFO registers happens. This code is interesting because of the way the memory address for the SPI_DR register is calculated and derefenced.

```c
static inline void s_spi_raw_fifo_push8(SPI_TypeDef* SPIx, uint8_t data)
{
  // SPIx is a pointer, so will a contain a memory address of the SPI device
  // 0x0C is the offset of the Data Register with respect to the SPI device.
  // The casting "(uint32_t)" is necessary here because SPIx is a pointer address,
  // but it's trying to save it into a non-pointer variable. The compiler won't allow
  // this unless you force it by doing the casting.
  const uint32_t spix_dr = (uint32_t)SPIx + 0x0C;
  // spix_dr is the address of the SPIX_DR data register
  // Reading from this address will return the oldest frame of data from the Rx FIFO
  // Writing to this address will write to the end of the Tx FIFO

  // In this case, we are writing to it
  *(__IO uint8_t *) spix_dr = data;
  // *(__IO uint8_t *) spix_dr = data means:
  // - (__IO uint8_t *) spix_dr: cast spidx_dr to a pointer (*) of type __IO uint8_t
  // - first "*": derreference the pointer we just casted and
  // - "= data": assign the value of data to the derefferenced location
  // In other words, it's equivalent to:
  // __IO uint8_t *ptr_spix_dr = (__IO uint8_t *) spix_dr; // again, casting to change from uint to *uint
  // *ptr_spix_dr = data;
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
With the setup covered, we can begin building an application that will allow us to connect to the NTS1 controller, set our ACK pin to 1, and start receiving data.

### ESP32 programming with platformio and espidf
I'll base my test application on the ESP32 MCU, using platformio with the ESP IDF. This means that I won't be producing any Arduino code for now, and I'll program directly against the ESP32 specific libraries. Doing this will simplify the design of the test app, given that most MCUs come with development kits that contain working examples for using their different peripherals, SPI included.

I'll use platformio instead of the barebones ESP IDF because a package manager + build tool that does most of the heavy lifting out of the box is a great addition to any toolbelt. It'll simplify installing and using the tools needed to flash the chip and install vendor libraries.

You can find all the information you need on how to start a new platformio project for the ESP32 here: https://docs.platformio.org/en/latest/platforms/espressif32.html

### Identifying the NTS1 pins
Before we start building the test app, we need to identify the pins the NTS1 uses for SPI communication and ACK.

Looking at the included schematic for the reference panel, we can see that the headers used to connect to the NTS1 one are labelled "MAIN CONNECTOR". There are 2 rows of pins, labelled CN2 and CN7, and their pins are named as follows:
Header | Pin N | Name       | STM32 Pin
-------| ----- | ---------- | ---------
CN2    | 1     | GND        | -
CN2    | 2     | 3V3        | -
CN2    | 5     | PANEL_ACK  | PB12
CN2    | 7     | GND        | -
CN7    | 1     | GND        | -
CN7    | 2     | CK_PNL     | PB13 (SPI2_SCK)
CN7    | 3     | RX_PNL     | PB15 (SPI2_MOSI)
CN7    | 4     | TX_PNL     | PB14 (SPI2_MISO)
CN7    | 5     | RESET_PNL  | -
CN7    | 6     | BOOT_PNL   | -
CN7    | 7     | GND        | -

From looking at the Gerber files (and a video that I can't seem to find now), it seems CN2 corresponds to the left side header in the NTS1. Pins start from 1 to 7, bottom side first (closer to the headphone jack). Testing with an LED between the left hand side bottommost pin (CN21) and other pins confirms this hypothesis.

So, we'll use the following pins (in the NTS1):
- LHS2: 3V3
- LHS5: ACK
- RHS1: GND 
- RHS2: SPI_CLK
- RHS3: SPI_RX (MOSI)
- RHS4: SPI_TX (MISO)

ESP32 (Sparkfun Thing plus):
NTS1   | GPIO    | Name      
-------| ------- | --------
LHS2   | 3V3     | 3V3     
LHS5   | 22      | PANEL_ACK  
RHS1   | GND     | GND      
RHS2   | 18      | CK_PNL   
RHS3   | 23      | MOSI     
RHS4   | 19      | MISO     

Using SPI3 (VSPI) as slave.

Header | Pin N | Name       | STM32 Pin
-------| ----- | ---------- | ---------
CN2    | 1     | GND        | -
CN2    | 2     | 3V3        | -
CN2    | 5     | PANEL_ACK  | PB12
CN2    | 7     | GND        | -
CN7    | 1     | GND        | -
CN7    | 2     | CK_PNL     | PB13 (SPI2_SCK)
CN7    | 3     | RX_PNL     | PB15 (SPI2_MOSI)
CN7    | 4     | TX_PNL     | PB14 (SPI2_MISO)
CN7    | 5     | RESET_PNL  | -
CN7    | 6     | BOOT_PNL   | -
CN7    | 7     | GND        | -

### Finding the clock frequency
The NTS1 starts the clock when the ACK pin is on. We can use this to obtain the clock frequency by using a logic analyzer and PulseView or an oscilloscope. I went with a very cheap 24MHz logic analyzer (10€ with fast shipping, cheaper if you're willing to wait and order in AliExpress or Ebay).

I set the NTS1 ACK pin to +3v3, and connected the logic analyzer to the CLK pin. The period of the clock is 1us, which means that the frequency is 1MHz. This is good news because:
- The ESP32 can accept clock frequencies from 2.5KHz to 10MHz.
- The speed is not very high, so most oscilloscopes and logic analyzers should be able to deal with it easily.

### Modifying the "SPI slave" example project
The ESP32 IDF comes with an example for SPI slave mode, which we'll use as a base for the test application.
https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_slave/receiver/main

This the modified example. It uses the ESP32 IDF spi-slave driver to configure the device and establish a SPI connection. Things to take into account:
- It uses the IO_MUX pins for SPI, so that communication doesn't get delayed by the GPIO matrix. This seems important, I tried without it and the messages didn't produce anything clear.
- We are changing the default bit order to LSB, like the reference panel does.
- The example also includes a handshake/ACK routine, so that's nice, although it might work in a different way. Using the default for now seems to produce some results.
- The ESP32 SPI driver doesn't have a "software" CS pin, nor does it work without it. I couldn't identify a CS pin in the NTS1, so instead I leave it always on by enabling the pulldown resistor for that pin.

`examples/0-log-received.main.c`
```c
...
while(1){
        //Clear receive buffer
        memset(s_spi_rx_buf, 0x0, SPI_RX_BUF_SIZE);
        memset(s_spi_tx_buf, 0x0, SPI_TX_BUF_SIZE);

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
}
...
```

### The Rx handler
In order to understand the messages that the NTS1 will be sending, we need to start looking at the actual message handler:
```c

// These are constants that will be used in the method

#define PANEL_ID_MASK    0x38  // Bits 3-5 // 00111000 
#define PANEL_CMD_EMARK  0x40  // Bit  6   // 01000000
#define PANEL_START_BIT  0x80  // Bit  7   // 10000000

static uint8_t  s_panel_id = PANEL_ID_MASK; // Bits 3-5 "ppp"="111"
static uint8_t  s_dummy_tx_cmd = (PANEL_ID_MASK + 0xC7); // B'11ppp111;

enum {
  k_rx_cmd_event = 0x84U, // 10000100
  k_rx_cmd_param = 0x85U, // 10000101
  k_rx_cmd_other = 0x86U, // 10000110
  k_rx_cmd_dummy = 0x87U  // 10000111
};

// This is the handler itself
static void s_rx_msg_handler(uint8_t data)
{
  // data is 1 byte of the input buffer
  // If data byte starts with 10000000
  if (data >= 0x80) {
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
      s_panel_rx_status = data & ~PANEL_ID_MASK; // produces 1x000xxx
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
  // otherwise, command is discarded
  
  // Stored status byte
  const uint8_t active_cmd = s_panel_rx_status;
  
  // Now we process bytes that don't start with 1
  switch (active_cmd) {
  case k_rx_cmd_event: // 1x111 100
    // ...does stuff
    break;
  case k_rx_cmd_param: // 1x111 101
    // ...does stuff
    break;
  case k_rx_cmd_other: // 1x111 110
    // ...does stuff
    break;
  case k_rx_cmd_dummy: // 10000111
    // continues to default
  default:
    // resets
    s_panel_rx_status = 0;    // Clear save status
    s_panel_rx_data_cnt = 0; // Initialize data count
    break;
  }
}
```

It looks like it reads the line byte by byte, waiting for a valid status byte to be received, which must have a predefined format, and then continues to process more bytes for each of those command. Once the handler has a valid status byte saved, it continues to process the following bytes, differently for every command.

### Command messages
The one thing command messages do have in common is the format of the first bytes:
```
      /*++++++++++++++++++++++++++++++++++++++++++++++
        CMD4 : Event
        1st    :[1][0][ppp][100]
        2nd    :[0][sssssss] Size
        3rd    :[0][eeeeeee] Event ID
        4th    :[0][ddddddd] Data word
        ...
        +++++++++++++++++++++++++++++++++++++++++++++*/
```
- 1st is always the status byte/command selector
- 2nd is always the size of the message
- 3rd is the sub-command or event ID
- 4th is data associated to that command

### Finding status bytes
As we saw before, the handler follows these rules to tell apart status bytes from other bytes:
- Statuses start with 1
- The very next bit is always discarded.
- It accepts any message that conforms to this format: 1x111xxx.
- It accepts one particular message (1x111110), that is for some reason treated differently.

We can translate the hex strings into binary bytes and look for 1x111xxx.To make it easier, we can use a regex for that: /1[0-1]111[0-1]{3}/. 

However, in my many attempts to receive something intelligible from the NTS1, I failed misserably. I supect my SPI timing is a bit off. I did get to send note-on messages and make it sound, though.

#### A tribute to lost time
I'm making an aside here to say that, when I got to this point of the analysis (reading the data from the NTS1), I lost a couple of evenings because, of pin misnomer (MISO for MOSI etc.), and because, apparently, you cannot simply connect your ground to any of the available grounds. I only managed to make it work when I connected the RHS pin 1 ground to my ground. The rest of them (which I assumed should have worked the same), did not work properly. I guess, even though they are sent to ground **in the reference panel**, they are not connected to ground in the NTS1.

So, here, toast with me to lost time and lessons (eventually) learned.

This is also why I went into detail about the Commands before. Given that we are not actually getting any interesting commands, the next thing to do is to test that we can send commands to the NTS1 and see if it does anything.

### Sending commands
Once we have the base SPI connection working, we can start sending messages to test that the NTS1 receives them. 

We saw in the Interrupt handler that the reference panel writes commands from its software buffer to the TX FIFO. These messages are put into the software user by the user, by calling one of the public methods in the `nts1_iface.h` class, or one of its equivalents in the arduino library (`nts-1.h`). We can follow the trace of one of these methods grab an example message. The most obvious one to try first is the `note_on` message, since that will make the NTS1 bleep and bloop for us.

```c

nts1_status_t nts1_note_on(uint8_t note, uint8_t velo) {
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_note_on;
  event.msb = note & 0x7F;
  event.lsb = velo & 0x7F;
  return nts1_send_event(&event); 
}
static inline nts1_status_t nts1_send_event(nts1_tx_event_t *event) {
  return nts1_send_events(event, 1);
}
 
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

static uint8_t s_tx_cmd_event(const nts1_tx_event_t *event, uint8_t endmark) 
{
  assert(event != NULL);
  if (!s_spi_chk_tx_buf_space(4)) 
     return false;
  // if (s_panel_id & PANEL_ID_MASK) + (endmark) > 0, add PANEL_CMD_EMARK bit to the 
  // command byte
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_event | PANEL_CMD_EMARK) : k_tx_cmd_event; 
  s_spi_tx_buf_write(cmd);
  // mask out the Most Significant bit, just in case I guess, so it
  // doesn't get confused with a command byte
  s_spi_tx_buf_write(event->event_id & 0x7F);
  s_spi_tx_buf_write(event->msb & 0x7F);
  s_spi_tx_buf_write(event->lsb & 0x7F);
  return true;
}
// writes the byte to the buffer at the appropiate position
// and moves the buffer pointers
static void s_spi_tx_buf_write(uint8_t data)
{
  s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_widx] = data;
  s_spi_tx_widx = SPI_BUF_INC(s_spi_tx_widx, SPI_TX_BUF_SIZE);
}

```

The logic here is straight forward: a call to the nts1_note_on(note, velocity) method will start chain of methods that will format the message appropiately and manage the internal status of the firmware.

The bytes that end up in the buffer are:
- s_spi_tx_buf[0] = 196; // note on command, with ENDMARK
- s_spi_tx_buf[1] = 1; // size, pressumably  // TODO this seems wrong
- s_spi_tx_buf[2] = note; // note number (0-127)
- s_spi_tx_buf[3] = 107;  // velocity

### Putting it together
With this we can test our assumptions and build a firmware that will send note-on mesages from our controller.

`examples/1-note-on.main.c`
```c
...
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

        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
...
```
------

## Next steps
The code as is is not very efficient, and I get glitches from time to time. I'm not convinced that I'm getting the messages as they are being sent. I'm waiting for a logic analyzer to arrive so that I can debug the SPI timing.

- ~~Debug SPI timing.~~ Now working properly (in src/main.c)
- Fix the examples with the latest changes.
- Add some info on how I used Pulseview to debug the wiring.
- Add a proper diagram for the connections.
- Test whether making all the connections present in the schematics is necessary.
- Test more messages.
- Work on separating the not-STM32-specific stuff into another class, and using the resulting library in the project.
- Maybe try to propose the changes to the main `nts-1-customizations` repo? Or publish this as a separate library so that it can be used in ESP32 platformio projects.
- Build an instrument with some sensors and a screen as a proof of concept.

-----

# Notes for people not used to C
Like me.

- `#define X`: this is a **preprocessor** macro, meaning that anything you define like this will not be stored in a variable. Instead the **preprocessor** will change references to it into the actual value in your code before compiling it.
- `int* pc; int c; c = 5; pc = &c;`: assign to `pc` the address of the variable `c`.
- `int* pc, c; c = 5; pc = &c; printf("%d", *pc);`: Output: 5. `*pc` derreferences (get the value) of the memory location pointed by `pc`.
- `int* pc, c; c = 5; pc = &c; *pc = 1; printf("%d", *pc); /* Ouptut: 1 */ printf("%d", c); // Output: 1`: you can write to the location pointed by a pointer by using derreference assignation.
- `int* pc = 0xFFu; int c; c = (int) pc; int* p2 = &c; printf("%x, %x", c, *p2) //Out: FF, FF`: You cannot store a memory address directly in a non-pointer variable, you need to cast it.
- `*((uint_32 int *) non_pointer_with_address) = value;`: Writing to the memory address stored in a non-pointer variable.
- `int p[] = {'a','b'}; p[1] == *(p + 1)`: An array is a pointer to a location in memory.
- `int * REG_GPIO_BASE = 0x0u; int REG_GPIO_SETTINGS; REG_GPIO[REG_GPIO_SETTINGS] |= (1u << 1)`: You can use array notation on a pointer to access offsets from a location in memory.
- `volatile int x`. Volatile variables tell the compiler that the value of the variable might change without the program making it so (eg: a register that contains the state of a gpio pin). This avoids compiler optimizations that might bypass storing the variable in memory, and thus break your code.
- Logic vs arithmetic (right) shifting: negative signed numbers (int8_t) behave differently from unsigned numbers (uint8_t) when performing bitwise right shifting.
- Logic right shifting: Unsigned numbers get logically shifted. `128 (10000000) >> 1` => `01000000`.
- Arithmetic right shifting: Signed negative numbers numbers get arithmetically shifted. `-64 (11000000) >> 1` => `11100000`.
- `(1u << 3)` => `00001000`: Single bit representation.
- `REG_X |= (1u << 3)`: sets the 4th bit (bit 3) of REG_X to 1.
- `REG_X &= ~(1u << 3)`: (un)sets the 4th bit (bit 3) of REG_X to 0.
- These last idioms are often optimized in the compiler into faster operations than doing the whole calculation, using a Bit Set or Bit Clear instructions instead of the operation, so it's preferrable to use them.
- Interrupts can happen between **processor** instructions. Meaning that even if you wrote a single statement, the processor can be still in the middle of excuting that statement when it gets interrupted, because your statement may get translated into several processor instructions. This is particularly important when setting bits in registers, as this often gets compiled into 3 instructions (or more).
- To avoid this happening when changing registers, you can use **atomic operations**. An assignment of a value that is equals or smaller (in bits) than the size of a single memory address, eg: 32bits for a a 32bit memory; is a atomic operation. This is not universal, C doesn't guarantee it.