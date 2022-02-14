//
// Created by Gleb Ryabtsev on 9/15/2021.
//

#include <Arduino.h>
#include <EADC.h>
#include "SMPS.h"
#include "LDO.h"

#define _CONNECTION_REQUEST_CMD 0x00
#define _SET_VOLTAGE_REQUEST_CMD 0X01
#define _READ_VOLTAGE_REQUEST_CMD 0x02
#define _READ_CURRENT_REQUEST_CMD 0x03
#define _CHANGE_CHANNEL_MODE_REQUEST_CMD 0x04
#define _CALIBRATAION_REQUEST_CMD 0x05

#define _LINEAR_CALIBRATION 0x00

#define _N_CHANNELS 2

#define _CHANNEL_MODE_DISABLED 0x00
#define _CHANNEL_MODE_STANDARD 0x01

//#define FEEDBACK_THRESHOLD_ENABLE

#define LDO_OUTPUT_COEFFICIENT (65536.0 * 6.0 / 2500.0)

/* FUNCTION PROTOTYPES */

void process_serial_request();

void handle_connection_request();

void handle_set_voltage_request();

void handle_read_voltage_request();

void handle_read_current_request();

void handle_change_channel_mode_request();

void handle_calibration_request();

void send_standard_acknowledgement();

void send_response(uint8_t *data, uint8_t length);

/* HARDWARE */

EADC adc = EADC{2, 13, 5};
SMPS smps = SMPS{8, 9};
LDO ldo = LDO{10};

uint16_t ldo_val[] = {0, 0};
EADC::VOLTAGE_CURRENT vc_measured[2];

/* STANDARD_MODE_OPERATION */

float v_target[] = {1.0, 1.0};
float k[] = {0.8, 0.8};

/* SERIAL CONNECTION */

boolean connected = false;
uint8_t serial_data_buffer[256];
uint8_t &ch_byte = serial_data_buffer[0];
uint8_t serial_data_length;


void setup() {
    Serial.begin(115200);

    EADC::configure_clock();
    //SPI.begin();
    _delay_ms(200);


    /*for (int i = 0; i < _N_CHANNELS; i++) {
        adc[i].setup();
        smps[i].setup();
        ldo[i].setup();
        smps[i].enable();
        smps[i].write(0xff);
        ldo[i].write(ldo_val[i]);
    }*/
    adc.setup();
    smps.setup();
    ldo.setup();

    smps.enable();
    smps.write(0xff);
    ldo.write(0);
//    _delay_ms(100); // just in case
//    adc[0].synchornize();
//    adc[1].synchornize();
}

void loop() {

//    while (!adc[1].available()); // Channel 2 was synchronized last, so when it's ready, Ch. 1 should also be ready
//
///* feedback control */
//    for(int i = 0; i < _N_CHANNELS; i++) {
//        vc_measured[i] = adc[i].read_voltage_current();
//        auto delta = (int16_t) (0.8 * 4.37 * (v_target[i] - vc_measured[i].voltage));
//
//#ifdef FEEDBACK_THRESHOLD_ENABLE
//        if(abs(delta) > 100) {
//#endif
//        ldo_val[i] += delta;
//        ldo[i].write(ldo_val[i]);
//#ifdef FEEDBACK_THRESHOLD_ENABLE
//        }
//#endif
//    }
//
//    if (Serial.available()) {
//        process_serial_request();
//    }
    ldo.write(10000);
//    ldo[1].write(3000);
    _delay_ms(4);
//    Serial.println("Hello");
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
        case _CHANGE_CHANNEL_MODE_REQUEST_CMD:
            handle_change_channel_mode_request();
            break;
        case _CALIBRATAION_REQUEST_CMD:
            handle_calibration_request();
            break;
        default:
            break;
    }
}

void handle_connection_request() {
    connected = true;
    send_standard_acknowledgement();
}

void handle_set_voltage_request() {
    if (!connected) return;
    if (ch_byte >= _N_CHANNELS) return;
    uint16_t v = (((uint16_t) serial_data_buffer[1]) << 8) | (serial_data_buffer[2]);
    v_target[ch_byte] = v;
    send_standard_acknowledgement();
}

void handle_read_voltage_request() {
    if (!connected) return;
    if (ch_byte >= _N_CHANNELS) return;
    send_response(reinterpret_cast<uint8_t *>(&vc_measured[ch_byte].voltage), 2);
}

void handle_read_current_request() {
    if (!connected) return;
    if (ch_byte >= _N_CHANNELS) return;
    send_response(reinterpret_cast<uint8_t *>(&vc_measured[ch_byte].current), 2);
}

void handle_change_channel_mode_request() {
    if (!connected) return;
    if (ch_byte >= _N_CHANNELS) return;
    switch(serial_data_buffer[1]) {
        case _CHANNEL_MODE_DISABLED:
            smps[ch_byte].disable();
            break;
        case _CHANNEL_MODE_STANDARD:
            smps[ch_byte].enable();
            break;
        default:
            return;
    }
    send_standard_acknowledgement();
}

void handle_calibration_request() {
    if (!connected) return;
    if (ch_byte >= _N_CHANNELS) return;
    switch (serial_data_buffer[1]) {
        case _LINEAR_CALIBRATION:
            // todo: implement calibration
            break;
        default:
            return;
    }
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
