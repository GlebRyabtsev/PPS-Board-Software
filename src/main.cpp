//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#include <Arduino.h>
#include <EADC.h>
#include "SMPS.h"
#include "DAC.h"

#define _CONNECTION_REQUEST_CMD 0x00
#define _SET_VOLTAGE_REQUEST_CMD 0X01
#define _READ_VOLTAGE_REQUEST_CMD 0x02
#define _READ_CURRENT_REQUEST_CMD 0x03
#define _UPDATE_CURRENT_CALIBRATION_REQUEST_CMD 0x04
#define _UPDATE_VOLTAGE_CALIBRATION_REQUEST_CMD 0x05

//#define FEEDBACK_THRESHOLD_ENABLE

#define LDO_OUTPUT_COEFFICIENT (65536.0 * 6.0 / 2500.0)

void process_serial_request();

void handle_connection_request();

void handle_set_voltage_request();

void handle_read_voltage_request();

void handle_read_current_request();

void send_standard_acknowledgement();

void send_response(uint8_t *data, uint8_t length);

void handle_update_current_calibration_request();

EADC adc1 = EADC{2, 13, 5};
SMPS smps1 = SMPS{8, 9};
DAC dac1 = DAC{10};

boolean connected = false;
uint8_t serial_data_buffer[256];
uint8_t serial_data_length;

float v_target_1 = 1000;
float v_target_2 = 0;

float k1, k2 = 0.8;

uint16_t ldo1, ldo2 = 0;

void setup() {
    Serial.begin(115200);
    EADC::configure_clock();
    //SPI.begin();
    _delay_ms(200);

    adc1.setup();
    smps1.setup();
    dac1.setup();

    smps1.enable();
    smps1.write(0xff);
    dac1.write(ldo1);
}

EADC::VOLTAGE_CURRENT vc_measured_1;

void loop() {
//    while (digitalRead(13));
//    /* feedback control */
//    vc_measured_1 = adc1.read_voltage_current();
//    auto delta = (int16_t) (0.8 * 4.37 * (v_target_1 - vc_measured_1.voltage));
//
//    //Serial.println(delta)
//#ifdef FEEDBACK_THRESHOLD_ENABLE
//    if(abs(delta) > 100) {
//#endif
//    ldo1 += delta;
//    dac1.write(ldo1);
//#ifdef FEEDBACK_THRESHOLD_ENABLE
//    }
//#endif

//    _delay_ms(4);
//    if (Serial.available()) {
//        process_serial_request();
//    }
    dac1.write(32000);
    _delay_ms(1000);
}

void process_serial_request() {
    // to be called on beginning of each request

    if (Serial.read() != 0xDD) {
        Serial.flush();
        return;
    }

    uint8_t cmd = Serial.read();
    serial_data_length = Serial.read();
    for (int i = 0; i < serial_data_length; i++) {
        if (!Serial.available()) break;
        serial_data_buffer[i] = Serial.read();
    }
    for (int i = serial_data_length; i < 256; i++) {
        serial_data_buffer[i] = 0;
    }

    Serial.flush();

    switch (cmd) {
        case _CONNECTION_REQUEST_CMD:
            handle_connection_request();
            break;
        case _SET_VOLTAGE_REQUEST_CMD:
            handle_set_voltage_request();
            break;
        case _READ_VOLTAGE_REQUEST_CMD:
            handle_read_voltage_request();
            break;
        case _READ_CURRENT_REQUEST_CMD:
            handle_read_current_request();
            break;
        case _UPDATE_CURRENT_CALIBRATION_REQUEST_CMD:
            handle_update_current_calibration_request();
        default:
            break;
    }
}

void handle_update_current_calibration_request() {
    if (!connected) return;
    if (serial_data_buffer[0] > 1) return;
    if (serial_data_buffer[0] == 0) {
        smps1.disable();
        _delay_ms(5000);

    }
}

void handle_connection_request() {
    connected = true;
    send_standard_acknowledgement();
}

void handle_set_voltage_request() {
    if (!connected) return;
    if (serial_data_buffer[0] > 1) return;
    uint16_t v = (((uint16_t) serial_data_buffer[1]) << 8) | (serial_data_buffer[2]);
    if (serial_data_buffer[0] == 0) v_target_1 = v;
    else v_target_2 = v;
//  Serial.print(v_target_1);
//  Serial.print(' ');
//  Serial.print(ldo1);
//  Serial.print(' ');
//  Serial.print(vc_measured_1.voltage);
//  Serial.print(' ');
//  Serial.println((int16_t)(0.8 * 4.37 * (v_target_1 - vc_measured_1.voltage)));
    send_standard_acknowledgement();
}

void handle_read_voltage_request() {
    if (!connected) return;
    if (serial_data_buffer[0] > 1) return;
    int16_t v = 0;
    if (serial_data_buffer[0] == 0) {
        v = (int16_t) (vc_measured_1.voltage);
    } else {
        v = 0;
    }
    send_response(reinterpret_cast<uint8_t *>(&v), 2);
}

void handle_read_current_request() {
    if (!connected) return;
    if (serial_data_buffer[0] > 1) return;
    int16_t c = 0;
    if (serial_data_buffer[0] == 0) {
        c = (int16_t) (vc_measured_1.current);
    } else {
        c = 0;
    }
    send_response(reinterpret_cast<uint8_t *>(&c), 2);
}

void send_standard_acknowledgement() {
    uint8_t data[] = {'A', 'C', 'K'};
    send_response(data, 3);
}

void send_response(uint8_t *data, uint8_t length) {
    Serial.write(0xEE);
    Serial.write(length);
    for (int i = 0; i < length; i++) {
        Serial.write(data[i]);
    }
}
