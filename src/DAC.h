#ifndef DAC_h
#define DAC_h

#include <Arduino.h>
#include <SPI.h>
#include "EADC.h"

class DAC
{
private:
    byte sync;
    static const SPISettings spi_settings;
    void write_reg(byte reg, uint16_t data);
public:

  DAC(const byte &sync);

  void setup();
  void write(uint16_t value);
};

#endif