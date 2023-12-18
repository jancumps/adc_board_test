#include "ads1115.h"

// configure for first found IC
bool ads1115::init(i2c_inst_t *i2c_port) { 
    bool found = false;
    uint8_t u;
    uint8_t buf[3] = {(uint8_t)reg::REG_CONFIG};
    // is the ADC chip installed?
    for (u = (uint8_t)addr::A_0; u < (uint8_t)addr::A_LAST; u++) { // test both board I2C addresses
        i2c_write_blocking(i2c_port, u, buf, 1, false);
        if (i2c_read_blocking(i2c_port, u, buf, 2, false) != PICO_ERROR_GENERIC) { 
            found = ads1115::init(i2c_port, (addr)u); // chip found!
            break;
        }
    }
    return found;
}

// configure for IC with provided address
bool ads1115::init(i2c_inst_t *i2c_port, addr address) { 
    this->i2c_port = i2c_port;
    this->address = address;
    return true;
}

// these build_xxx functions set up the local config register variables,
// for later programming into the ADC.

void ads1115::build_data_rate(data_rate dr) {
    this->dr = dr;
    adc_confreg[1] &= ~0xE0; // clear the DR bits
    adc_confreg[1] |= ((uint8_t)dr << 5); // set the DR bits
}

void ads1115::build_gain(channel chan, gain gain) {
    adc_range[(uint8_t)chan] = adc_range_value[(uint8_t)gain];
    adc_gain_bits[(uint8_t)chan] = gain;
    adc_confreg[0] &= ~0x0E; // clear the PGA bits
    adc_confreg[0] |= ((uint8_t)gain << 1); // set the PGA bits
}

void ads1115::build_cont_conversion() {
    adc_confreg[1] &= ~0x01; // clear the single-shot mode bit
}

void ads1115::build_single_conversion() {
    adc_confreg[1] |= 0x01; // set the single-shot mode bit
}

// selects either the first or second channel on the ADC board
// this ends up programming the entire config register
// if do_single_conversion is 1 then a single conversion will be immediately started.
void ads1115::adc_set_mux(channel chan, bool do_single_conversion) {
    uint8_t mux;
    uint8_t buf[3];
    switch(chan) {
        case channel::CH_AIN1:
            mux = (uint8_t)diff::DIFF_0_1;
            break;
        case channel::CH_AIN2:
            mux = (uint8_t)diff::DIFF_2_3;
            break;
    }
    adc_confreg[0] &= ~0x70; // clear the MUX bits
    adc_confreg[0] |= (mux << 4); // set the MUX bits
    buf[0] = (uint8_t)reg::REG_CONFIG;
    buf[1] = adc_confreg[0];
    buf[2] = adc_confreg[1];
    if (do_single_conversion) {
        buf[1] |= 0x01; // set MODE bit to single-shot mode
        buf[1] |= 0x80; // set START bit to start conversion
    }
    i2c_write_blocking(i2c_port, (uint8_t)address, buf, 3, false);
}

void ads1115::adc_enable_ready() {
    uint8_t buf[3];

    buf[0] = (uint8_t)reg::REG_LO_THRESH;
    buf[1] = 0x00;
    buf[2] = 0x00; //Lo_thresh MS bit must be 0
    i2c_write_blocking(i2c_port, (uint8_t)address, buf, 3, false);

    buf[0] = (uint8_t)reg::REG_HI_THRESH;
    buf[1] = 0x80;
    buf[2] = 0x00; //Hi_thresh  MS bit must be 1
    i2c_write_blocking(i2c_port, (uint8_t)address, buf, 3, false);
}

void ads1115::bulk_read(uint16_t* buf, size_t len) {

    uint8_t* begin = (uint8_t*)buf;
    uint8_t* end = ((uint8_t*)buf) + len;

    uint8_t reg = (uint8_t)reg::REG_CONVERSION;
    i2c_write_blocking(i2c_port, (uint8_t)address, &reg, 1, false);

    set_data_ready(false);
    while (begin < end) {
        if (is_data_ready()) {
            set_data_ready(false);
            i2c_read_blocking(i2c_port, (uint8_t)address, begin, 2, false);
            begin += 2;
        }
    }
}

void ads1115::start_single_conversion() {
    uint8_t buf[3];
    build_single_conversion();
    buf[0] = (uint8_t)reg::REG_CONFIG;
    buf[1] = adc_confreg[0];
    buf[2] = adc_confreg[1];
    i2c_write_blocking(i2c_port, (uint8_t)address, buf, 3, false);
}

// read the conversion register
uint16_t ads1115::adc_raw_diff_result() {
    uint16_t meas;
    uint16_t buf;
    uint8_t* buf8_ptr = (uint8_t*)&buf;
    *buf8_ptr = (uint8_t)reg::REG_CONVERSION;
    i2c_write_blocking(i2c_port, (uint8_t)address, buf8_ptr, 1, false);
    i2c_read_blocking(i2c_port, (uint8_t)address, buf8_ptr, 2, false);
    return(meas);
}
