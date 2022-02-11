#ifndef ads131m02_h
#define ads131m02_h

#include <Arduino.h>
#include <SPI.h>

class EADC {
public:
    struct ADC_RAW_VALUE {
        uint32_t ch0;
        uint32_t ch1;

        ADC_RAW_VALUE(uint32_t ch0, uint32_t ch1) : ch0(ch0), ch1(ch1) {}
    };

    struct VOLTAGE_CURRENT {
        int16_t voltage, current;
    };

    struct CALIBRATION {
        float m, n; // adjusted value = m*(naive value) + n;
    };

private:
    void write_reg(uint8_t reg, uint16_t value);

    uint8_t cs, drdy, sync_rst;
    static const SPISettings spi_settings;
    CALIBRATION voltage_cal = {14400.0 / ((float) 0x7FFFFF), 0.0}; // in volts, gain 1, ratio 12
    CALIBRATION current_cal = {6000.0 / ((float) 0x7FFFFF), -357.0}; // in amps, gain 2
public:
    EADC(uint8_t cs, uint8_t drdy, uint8_t syncRst) : cs(cs), drdy(drdy), sync_rst(syncRst) {}

    void setup();

    static void configure_clock();

    ADC_RAW_VALUE read();

    VOLTAGE_CURRENT read_voltage_current();

    bool available();

    void synchornize();
};


#endif