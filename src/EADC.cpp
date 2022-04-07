#include "EADC.h"
#include <spi_procedures.h>
#include <EEPROM.h>

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

#define _CAL_EEPROM_ADDR 0x00

const SPISettings EADC::spi_settings = SPISettings{250000, MSBFIRST, SPI_MODE1};
const EADC::Calibration EADC::default_voltage_cal{14400.0 / ((float) 0x7FFFFF), 0.0}; // gain 1, ratio 12 through resistor divider
const EADC::Calibration EADC::default_current_cal{6000.0 / ((float) 0x7FFFFF), -357.0}; // gain 2

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

    voltage_cal = load_calibration(CalibrationEEPROMOffset::VOLTAGE);
    current_cal = load_calibration(CalibrationEEPROMOffset::CURRENT);
}

void EADC::write_reg(uint8_t reg, uint16_t value) const {
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

EADC::ADCRawValue EADC::read() const {
    ADCRawValue adc_value = ADCRawValue(0, 0);

    spi_begin_transaction(spi_settings, cs);

    SPI.transfer16(_ADC_CMD_NULL);
    SPI.transfer(0x00);

    adc_value.ch0 = ((uint32_t) SPI.transfer(0x00)) << 16;
    adc_value.ch0 |= SPI.transfer16(0x0000);

    adc_value.ch1 = ((uint32_t) SPI.transfer(0x00)) << 16;
    adc_value.ch1 |= SPI.transfer16(0x0000);

    SPI.transfer(0x00);
    SPI.transfer16(0x0000);

    spi_end_transaction();

    return adc_value;
}

EADC::VoltageCurrent EADC::read_voltage_current() const {
    ADCRawValue raw = read();
    VoltageCurrent result{};

    int32_t voltage_reading, current_reading;

    if (raw.ch0 & (uint32_t) (1 << 23)) {
        // negative
        current_reading = ((int32_t) raw.ch0) - ((int32_t) 0xFFFFFF);
    } else {
        //positive
        current_reading = (int32_t) raw.ch0;
    }
    result.current = round(static_cast<float>(current_reading) * current_cal.m + current_cal.n);

    if (raw.ch1 & (uint32_t) (1 << 23)) {
        voltage_reading = ((int32_t) raw.ch1) - ((int32_t) 0xFFFFFF);
    } else {
        voltage_reading = raw.ch1;
    }
    result.voltage = round(static_cast<float>(voltage_reading) * voltage_cal.m + voltage_cal.n);

    return result;
}

bool EADC::available() const {
    return !digitalRead(drdy);
}

void EADC::synchronize() const {
    digitalWrite(sync_rst, 0);
    delayMicroseconds(10);
    digitalWrite(sync_rst, 1);
}

void EADC::use_default_calibration() {
    voltage_cal = default_voltage_cal;
    current_cal = default_current_cal;
}

void EADC::calibrate_voltage(int16_t _v0, int16_t _v1, int16_t _v_target) {
    auto v0 = static_cast<float>(_v0);
    auto v1 = static_cast<float>(_v1);
    auto v_target = static_cast<float>(_v_target);

    float x0, xv1, m_new, n_new;
    x0 = (v0 - default_voltage_cal.n) / default_voltage_cal.n;
    xv1 = (v_target - default_voltage_cal.n) / default_voltage_cal.m;

    m_new = v1 / (xv1 - x0);
    n_new = - x0 * m_new;

    voltage_cal.m = m_new;
    voltage_cal.n = n_new;

    save_calibration(CalibrationEEPROMOffset::VOLTAGE, voltage_cal);
}

void EADC::calibrate_current(int16_t _i0, int16_t _i1_real, int16_t _i1_measured) {
    auto i0 = static_cast<float>(_i0);
    auto i1_real = static_cast<float>(_i1_real);
    auto i1_measured = static_cast<float>(_i1_measured);

    float x0, xi1r, m_new, n_new;
    x0 = (i0 - default_current_cal.n) / default_current_cal.m;
    xi1r = (i1_measured - default_current_cal.n) / default_current_cal.m;

    m_new = i1_real / (xi1r - x0);
    n_new = - x0 * m_new;

    current_cal.m = m_new;
    current_cal.n = n_new;

    save_calibration(CalibrationEEPROMOffset::CURRENT, current_cal);
}

EADC::EADC(uint8_t cs, uint8_t drdy, uint8_t syncRst, uint8_t channel)
        : cs(cs), drdy(drdy), sync_rst(syncRst), channel(channel) {}

void EADC::save_calibration(EADC::CalibrationEEPROMOffset co, EADC::Calibration c) {
    EEPROM.put(_CAL_EEPROM_ADDR + co, c);
}

EADC::Calibration EADC::load_calibration(EADC::CalibrationEEPROMOffset co) {
    Calibration c{};
    EEPROM.get(_CAL_EEPROM_ADDR + co, c);
    return c;
}
