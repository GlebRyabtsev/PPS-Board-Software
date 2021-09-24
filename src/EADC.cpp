#include "EADC.h"
#include <spi_procedures.h>

#define _ADC_CMD_NULL     ((uint16_t) 0x0000)
#define _ADC_CMD_RESET    ((uint16_t) 0x0001)
#define _ADC_CMD_STANDBY  ((uint16_t) 0b00100010)
#define _ADC_CMD_WAKEUP   ((uint16_t) 0b00110011)
#define _ADC_CMD_LOCK     ((uint16_t) 0b0000010101010101)
#define _ADC_CMD_UNLOCK   ((uint16_t) 0b0000011001100110)
#define _ADC_CMD_PREG_MSK ((uint16_t) 0b1010000000000000)
#define _ADC_CMD_WREG_MSK ((uint16_t) 0b0110000000000000)

#define _ADC_REG_CLOCK 0x03
#define _ADC_REG_GAIN1 0x04

const SPISettings EADC::spi_settings = SPISettings{250000, MSBFIRST, SPI_MODE1};

void EADC::setup() {
  pinMode(cs, OUTPUT);
  pinMode(drdy, INPUT);
  pinMode(sync_rst, OUTPUT);
  pinMode(MOSI, OUTPUT);
  digitalWrite(cs, 1);
  digitalWrite(sync_rst, 1);
  digitalWrite(MOSI, 1);
  /* set gains: 1 for the voltage divider, 2 for the current sensor */
  write_reg(_ADC_REG_GAIN1, (uint16_t) 0b1);
  /* set oversampling ratio to 64384, corresponds to 250 SPS */
  write_reg(_ADC_REG_CLOCK, (uint16_t) ((1 << 9) | (1 << 8) | (0b111 << 2) | (1 << 1)));
}

void EADC::write_reg(uint8_t reg, uint16_t value) {
  uint16_t cmd_word = (_ADC_CMD_WREG_MSK | ((uint16_t) reg << 7));
  spi_begin_transaction(spi_settings, cs);

  SPI.transfer16(cmd_word);
  SPI.transfer(0x00);

  SPI.transfer16(value);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  spi_end_transaction();
}


void EADC::configure_clock() {
  DDRB |= (1 << DDB7);
  TCCR0A = ((1 << COM0A0) | (1 << WGM01));
  TCCR0B = (1 << CS00);
  OCR0A = 7; // 1 (MHz) output
}

EADC::ADC_RAW_VALUE EADC::read() {
  ADC_RAW_VALUE adc_value = ADC_RAW_VALUE(0, 0);

  spi_begin_transaction(spi_settings, cs);

  SPI.transfer16(_ADC_CMD_NULL);
  SPI.transfer(0x00);

  adc_value.ch0 = ((uint32_t)SPI.transfer(0x00)) << 16;
  adc_value.ch0 |= SPI.transfer16(0x0000);

  adc_value.ch1 = ((uint32_t)SPI.transfer(0x00)) << 16;
  adc_value.ch1 |= SPI.transfer16(0x0000);

  SPI.transfer(0x00);
  SPI.transfer16(0x0000);

  spi_end_transaction();

  return adc_value;
}

EADC::VOLTAGE_CURRENT EADC::read_voltage_current() {
  ADC_RAW_VALUE raw = read();
  VOLTAGE_CURRENT result;

  if(raw.ch0 & (uint32_t)(1<<23)) {
    // negative
    result.current = (float)((int32_t) raw.ch0) - ((int32_t)0xFFFFFF);
  } else {
    //positive
    result.current = (float) raw.ch0;
  }
  result.current = result.current * current_cal.m + current_cal.n;

  if(raw.ch1 & (uint32_t)(1<<23)) {
    result.voltage = (float)((int32_t) raw.ch1) - ((int32_t)0xFFFFFF);
  } else {
    result.voltage = (float) raw.ch1;
  }
  result.voltage = result.voltage * voltage_cal.m + current_cal.n;

  return result;
}
