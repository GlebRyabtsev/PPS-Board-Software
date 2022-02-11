//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#include "SMPS.h"
#include "spi_procedures.h"

const SPISettings SMPS::spi_settings = SPISettings{100000, MSBFIRST, SPI_MODE0};

SMPS::SMPS(const uint8_t &en, const uint8_t &cs) : en(en), cs(cs) {}

void SMPS::setup() {
    pinMode(cs, OUTPUT);
    pinMode(en, OUTPUT);
    digitalWrite(cs, 1);
    digitalWrite(en, 0);
}

void SMPS::enable() {
    digitalWrite(en, 1);
}

void SMPS::disable() {
    digitalWrite(en, 0);
}

void SMPS::write(uint8_t value) {
    spi_begin_transaction(spi_settings, cs);
    SPI.transfer(value);
    spi_end_transaction();
}

