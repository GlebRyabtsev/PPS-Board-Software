//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#ifndef BOARD_SOFTWARE_SMPS_H
#define BOARD_SOFTWARE_SMPS_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>


class SMPS {
private:
  uint8_t en, cs;
  static const SPISettings spi_settings;

public:
  SMPS(const uint8_t &en, const uint8_t &cs);
  void setup();
  void enable();
  void disable();
  void write(uint8_t value);
};


#endif //BOARD_SOFTWARE_SMPS_H
