//#include <Adafruit_SPIDevice.h>
#include <Arduino.h>

#define SPI_PIN_CS 10
#define SPI_PIN_CLOCK 10
#define SPI_PIN_MISO 10
#define SPI_PIN_MOSI 10
#define SPI_BITORDER SPI_BITORDER_LSBFIRST
#define SPI_MODE SPI_MODE3
// I assume the STM 32 impl is using the internal clock (HSI)
// which runs at 8 MHz. The code is using SPI_BAUDRATEPRESCALER_2
// which divides the clock source in half
#define SPI_CLOCK_RATE 4000000

// Adafruit_SPIDevice spi_dev = Adafruit_SPIDevice(
//   SPI_PIN_CS, SPI_PIN_CLOCK, SPI_PIN_MISO, SPI_PIN_MOSI,
//   100000, SPI_BITORDER, SPI_MODE);

void setup() {
    while (!Serial) {
        delay(10);
    }
    Serial.begin(115200);
    Serial.println("SPI device mode test");

    if (!nts1_init()) {
        Serial.println("Could not initialize SPI device");
        while (1);
    }
}

void loop() {
    Serial.println("\n\nTransfer test");
    for (uint16_t x=0; x<=0xFF; x++) {
        uint8_t i = x;
        Serial.print("0x"); Serial.print(i, HEX);
        //spi_dev.read(&i, 1, i);
        Serial.print("/"); Serial.print(i, HEX);
        Serial.print(", ");
        delay(25);
    }
}

uint8_t nts1_init(){
  
}