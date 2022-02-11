#include "LDO.h"
#include "spi_procedures.h"

#define DAC_GAIN_REG (byte) 0x04
#define DAC_DATA_REG (byte) 0x08

const SPISettings LDO::spi_settings = SPISettings{100000, MSBFIRST, SPI_MODE1};

void LDO::setup() {
    pinMode(sync, OUTPUT);
    digitalWrite(sync, 1);
    _delay_ms(1);
    write_reg(DAC_GAIN_REG, 0x0000);
}

void LDO::write_reg(byte reg, uint16_t data) {
    spi_begin_transaction(spi_settings, sync);
    SPI.transfer(reg);
    SPI.transfer16(data);
    spi_end_transaction();
}

void LDO::write(uint16_t value) {
    write_reg(DAC_DATA_REG, value);
}

LDO::LDO(const byte &sync) : sync(sync) {}
