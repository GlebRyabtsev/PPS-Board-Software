//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#ifndef BOARD_SOFTWARE_SPI_PROCEDURES_H
#define BOARD_SOFTWARE_SPI_PROCEDURES_H

#include <Arduino.h>
#include <SPI.h>

void spi_begin_transaction(SPISettings settings, uint8_t cs);

void spi_end_transaction();

#endif //BOARD_SOFTWARE_SPI_PROCEDURES_H
