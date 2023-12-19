#include "ads1115.h"

// configure for first found IC
bool ads1115::init(i2c_inst_t *i2c_port) { 
    bool found = false;
    uint8_t u;
    uint8_t buf[3] = {(*this)(reg::REG_CONFIG)};
    // is the ADC chip installed?
    for (u = (*this)(addr::A_0); u < (*this)(addr::A_LAST); u++) { // test both board I2C addresses
        i2c_write_blocking(i2c_port, u, buf, 1, false);
        if (i2c_read_blocking(i2c_port, u, buf, 2, false) != PICO_ERROR_GENERIC) { 
            found = ads1115::init(i2c_port, (*this)(u)); // chip found!
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
    adc_confreg[1] |= ((*this)(dr) << 5); // set the DR bits
}

void ads1115::build_gain(channel chan, gain gain) {
    adc_range[(*this)(chan)] = adc_range_value[(*this)(gain)];
    adc_gain_bits[(*this)(chan)] = gain;
    adc_confreg[0] &= ~0x0E; // clear the PGA bits
    adc_confreg[0] |= ((*this)(gain) << 1); // set the PGA bits
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
            mux = (*this)(diff::DIFF_0_1);
            break;
        case channel::CH_AIN2:
            mux = (*this)(diff::DIFF_2_3);
            break;
    }
    adc_confreg[0] &= ~0x70; // clear the MUX bits
    adc_confreg[0] |= (mux << 4); // set the MUX bits
    buf[0] = (*this)(reg::REG_CONFIG);
    buf[1] = adc_confreg[0];
    buf[2] = adc_confreg[1];
    if (do_single_conversion) {
        buf[1] |= 0x01; // set MODE bit to single-shot mode
        buf[1] |= 0x80; // set START bit to start conversion
    }
    i2c_write_blocking(i2c_port, (*this)(address), buf, 3, false);
}

void ads1115::adc_enable_ready() {
    uint8_t buf[3];

    buf[0] = (uint8_t)reg::REG_LO_THRESH;
    buf[1] = 0x00;
    buf[2] = 0x00; //Lo_thresh MS bit must be 0
    i2c_write_blocking(i2c_port, (*this)(address), buf, 3, false);

    buf[0] = (uint8_t)reg::REG_HI_THRESH;
    buf[1] = 0x80;
    buf[2] = 0x00; //Hi_thresh  MS bit must be 1
    i2c_write_blocking(i2c_port, (*this)(address), buf, 3, false);
}

void ads1115::start_single_conversion() {
    uint8_t buf[3];
    build_single_conversion();
    buf[0] = (uint8_t)reg::REG_CONFIG;
    buf[1] = adc_confreg[0];
    buf[2] = adc_confreg[1];
    i2c_write_blocking(i2c_port, (*this)(address), buf, 3, false);
}

// read the conversion register
uint16_t ads1115::adc_raw_diff_result() {
    uint16_t buf;
    uint8_t* buf8_ptr = (uint8_t*)&buf;
    *buf8_ptr = (uint8_t)reg::REG_CONVERSION;
    i2c_write_blocking(i2c_port, (*this)(address), buf8_ptr, 1, false);
    i2c_read_blocking(i2c_port, (*this)(address), buf8_ptr, 2, false);
    return(buf);
}
