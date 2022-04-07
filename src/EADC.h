#ifndef ads131m02_h
#define ads131m02_h

#include <Arduino.h>
#include <SPI.h>

class EADC {
public:
    struct ADCRawValue {
        uint32_t ch0;
        uint32_t ch1;

        ADCRawValue(uint32_t ch0, uint32_t ch1) : ch0(ch0), ch1(ch1) {}
    };

    struct VoltageCurrent {
        int16_t voltage, current;  // voltage and current in mv, ma
    };

    struct Calibration {
        float m, n; // adjusted value = m*(naive value) + n; (mv, ma)
    };

    enum CalibrationEEPROMOffset {
        VOLTAGE = 0 * sizeof(Calibration),
        CURRENT = 1 * sizeof(Calibration)
    };

    static const Calibration default_voltage_cal; // todo: clang doesn't like that
    static const Calibration default_current_cal;

    EADC(uint8_t cs, uint8_t drdy, uint8_t syncRst, uint8_t channel);

    void setup();
    static void configure_clock();
    ADCRawValue read() const;
    VoltageCurrent read_voltage_current() const;
    bool available() const;
    void synchronize() const;
    void use_default_calibration();
    void calibrate_voltage(int16_t v0, int16_t v1, int16_t v_target);
    void calibrate_current(int16_t i0, int16_t i1_real, int16_t i1_measured);

private:
    void write_reg(uint8_t reg, uint16_t value) const;

    const uint8_t cs, drdy, sync_rst;
    static const SPISettings spi_settings;  // todo: clang sees a problem

    const uint8_t channel;

    void save_calibration(CalibrationEEPROMOffset co, Calibration c);
    Calibration load_calibration(CalibrationEEPROMOffset co);

    Calibration voltage_cal{default_voltage_cal};
    Calibration current_cal{default_current_cal};
};


#endif