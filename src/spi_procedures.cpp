//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#include <spi_procedures.h>

uint8_t chip_select;

void spi_begin_transaction(SPISettings settings, uint8_t cs) {
    chip_select = cs;
    digitalWrite(MOSI, 0);
    SPI.begin();
    SPI.beginTransaction(settings);
    digitalWrite(chip_select, 0);
}

void spi_end_transaction() {
    digitalWrite(chip_select, 1);
    SPI.endTransaction();
    SPI.end();
    digitalWrite(MOSI, 1);
}