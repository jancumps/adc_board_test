/************************************************************************
 * adc_board_test
 * main.cpp
 * rev 1.0 - shabaz - oct 2023
 ************************************************************************/

// ************* header files ******************
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/i2c.h"

#include "hardware/irq.h"


// ***************** defines *******************
// buttons
#define BTN_EURO 27
// If a WaveShare Pico LCD 1.3 display is fitted, these pins are connected to buttons:
#define BTN_DISP_A 15
#define BTN_DISP_B 17
#define BTN_DISP_X 19
#define BTN_DISP_Y 21
#define BTN_DISP_LEFT 16
#define BTN_DISP_RIGHT 20
#define BTN_DISP_UP 2
#define BTN_DISP_DOWN 18
#define BTN_DISP_CENTER 3
#define PRESSED(btn) (gpio_get(btn)==0)
#define UNPRESSED(btn) (gpio_get(btn)!=0)
// I2C
// set this value to 0 or 1 depending on 0-ohm resistor positions on ADC board.
// set the value to 0 if the resistors are at the I2C0 labeled position,
// set the value to 1 if the resistors are at the unlabeled position (which will be I2C1)
// if the user did not set a selection, select I2C1 (shabaz solder instructions default)
#ifndef I2C_PORT_SELECTED
#define I2C_PORT_SELECTED 1
#endif // I2C_PORT_SELECTED
// these pins are supported by the ADC board:
#define I2C_SDA0_PIN 4
#define I2C_SCL0_PIN 5
#define I2C_SDA1_PIN 6
#define I2C_SCL1_PIN 7

// ADS1115 definitions
// I2C ADDR0 is used if the ADDR link is unshorted
// I2C ADDR1 is used if the ADDR link is shorted
#define ADC_ADDR0 0x49
#define ADC_ADDR1 0x48
// ADS1115 registers (4)
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03
// ADS1115 mux values
#define ADS1115_CH0 0x04
#define ADS1115_CH1 0x05
#define ADS1115_CH2 0x06
#define ADS1115_CH3 0x07
#define ADS1115_DIFF_0_1 0x00
#define ADS1115_DIFF_2_3 0x03
#define DR_8 0
#define DR_16 1
#define DR_32 2
#define DR_64 3
#define DR_128 4
#define DR_250 5
#define DR_475 6
#define DR_860 7
#define GAIN_6_144 0
#define GAIN_4_096 1
#define GAIN_2_048 2
#define GAIN_1_024 3
#define GAIN_0_512 4
#define GAIN_0_256 5
#define AIN1 0
#define AIN2 1
#define BOARD0 0
#define BOARD1 1
#define DO_SINGLE_CONVERSION 1

// misc
#define FOREVER 1


// ************ constants **********************
uint8_t adc_addr[2] = {ADC_ADDR0, ADC_ADDR1};
uint32_t conv_time_ms[8] = {125, 63, 32, 16, 8, 4, 3, 2}; // conversion time in us
double adc_range_value[8] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};


// ************ global variables ****************
i2c_inst_t *i2c_port;
uint8_t config_times = 0; // used to print a banner
uint8_t adc16Installed[2];
uint8_t adc_confreg[2][2]; // config register for ADC
uint8_t adc_data_rate[2] = {DR_860, DR_860};
uint8_t adc_gain_bits[2][2] = {{GAIN_2_048, GAIN_2_048}, /* BOARD0 */
                               {GAIN_2_048, GAIN_2_048}}; /* BOARD1 */
double adc_range[2][2] = {{2.048, 2.048}, /* BOARD0 */
                          {2.048, 2.048}}; /* BOARD1 */
double opamp_gain[2][2] = {{0.38298, 0.38298}, /* BOARD0 */
                           {0.38298, 0.38298}}; /* BOARD1 */

volatile bool rdy = false;


// ************ function prototypes *************
void adc_init(void); // initializes both ADC boards if present
void build_cont_conversion(uint8_t boardnum); // sets up the local config for continuous conversion
void build_single_conversion(uint8_t boardnum); // sets up the local config for single-shot conversion
void build_data_rate(uint8_t boardnum, uint8_t dr); // sets up the local config for data rate (e.g. DR_128)
void build_gain(uint8_t boardnum, uint8_t chan, uint8_t gain); // sets up the local config for gain (e.g. GAIN_2_048)
// adc_set_mux: programs the config and sets the channel (e.g. AIN1) and also starts the conversion
// if in continuous mode. Note: first result after changing mux in continuous mode will be invalid
// returns 0 if the ADC board was not installed. If do_single_conversion is 1, then the ADC is
// set to single-shot mode, and the conversion is started.
int adc_set_mux(uint8_t boardnum, uint8_t chan, uint8_t do_single_conversion);
int16_t adc_raw_diff_result(uint8_t boardnum); // returns the raw differential result (16-bit signed integer)
double to_volts(uint8_t boardnum, uint8_t chan, int16_t raw); // converts the raw result to volts


// ********** functions *************************


// general-purpose long delay timer if required
void
sleep_sec(uint32_t s) {
    sleep_ms(s * 1000);
}


// these build_xxx functions set up the local config register variables,
// for later programming into the ADC.
void build_cont_conversion(uint8_t boardnum) {
    adc_confreg[boardnum][1] &= ~0x01; // clear the single-shot mode bit
}
void build_single_conversion(uint8_t boardnum) {
    adc_confreg[boardnum][1] |= 0x01; // set the single-shot mode bit
}
void build_data_rate(uint8_t boardnum, uint8_t dr) {
    if (dr > 7) {
        printf("build_data_rate: invalid data rate\n");
        return;
    }
    adc_data_rate[boardnum] = dr;
    adc_confreg[boardnum][1] &= ~0xE0; // clear the DR bits
    adc_confreg[boardnum][1] |= (dr<<5); // set the DR bits
}
void build_gain(uint8_t boardnum, uint8_t chan, uint8_t gain) {
    if (gain > 5) {
        printf("build_gain: invalid gain\n");
        return;
    }
    adc_range[boardnum][chan] = adc_range_value[gain];
    adc_gain_bits[boardnum][chan] = gain;
    adc_confreg[boardnum][0] &= ~0x0E; // clear the PGA bits
    adc_confreg[boardnum][0] |= (gain<<1); // set the PGA bits
}

// selects either the first or second channel on the ADC board
// this ends up programming the entire config register
// returns 1 if successful, 0 otherwise.
// if do_single_conversion is 1 then a single conversion will be immediately started.
int adc_set_mux(uint8_t boardnum, uint8_t chan, uint8_t do_single_conversion) {
    uint8_t mux;
    uint8_t buf[3];
    if (adc16Installed[boardnum] == 0) {
        // board not installed
        return(0);
    }
    switch(chan) {
        case AIN1:
            mux = ADS1115_DIFF_0_1;
            break;
        case AIN2:
            mux = ADS1115_DIFF_2_3;
            break;
        default:
            printf("adc_set_mux: invalid channel number\n");
            break;
    }
    adc_confreg[boardnum][0] &= ~0x70; // clear the MUX bits
    adc_confreg[boardnum][0] |= (mux<<4); // set the MUX bits
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = adc_confreg[boardnum][0];
    buf[2] = adc_confreg[boardnum][1];
    if (do_single_conversion) {
        buf[1] |= 0x01; // set MODE bit to single-shot mode
        buf[1] |= 0x80; // set START bit to start conversion
    }
    i2c_write_blocking(i2c_port, adc_addr[boardnum], buf, 3, false);
    return(1);
}

int start_single_conversion(uint8_t boardnum) {
    uint8_t buf[3];
    if (adc16Installed[boardnum] == 0) {
        // board not installed
        return(0);
    }
    build_single_conversion(boardnum);
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = adc_confreg[boardnum][0];
    buf[2] = adc_confreg[boardnum][1];
    i2c_write_blocking(i2c_port, adc_addr[boardnum], buf, 3, false);
    return(1);
}

// read the conversion register
// result is 16-bit signed integer since the input can be negative or positive
int16_t adc_raw_diff_result(uint8_t boardnum) {
    uint16_t meas;
    uint16_t buf;
    uint8_t* buf8_ptr = (uint8_t*)&buf;
    if (adc16Installed[boardnum] == 0) {
        // board not installed
        return(0);
    }
    *buf8_ptr = ADS1115_REG_CONVERSION;
    i2c_write_blocking(i2c_port, adc_addr[boardnum], buf8_ptr, 1, false);
    i2c_read_blocking(i2c_port, adc_addr[boardnum], buf8_ptr, 2, false);
    meas = buf>>8 | buf<<8; // swap bytes
    return(meas);
}

double to_volts(uint8_t boardnum, uint8_t chan, int16_t raw) {
    double v;
    v = (double)raw * (adc_range[boardnum][chan] / 32768.0); // convert the raw value to volts present at the ADC input
    v = v / opamp_gain[boardnum][chan]; // adjust for opamp gain
    return(v);
}

void adc_init(void) {
    int i;
    uint8_t buf[3];
    for (i=0; i<2; i++) {
        adc16Installed[i] = 0;
        adc_confreg[i][0] = 0;
        adc_confreg[i][1] = 0;
    }
    // is the ADC chip installed?
    for (i=0; i<2; i++) { // test both board I2C addresses
        buf[0] = ADS1115_REG_CONFIG;
        i2c_write_blocking(i2c_port, adc_addr[i], buf, 1, false);
        if (i2c_read_blocking(i2c_port, adc_addr[i], buf, 2, false) == PICO_ERROR_GENERIC) { 
            adc16Installed[i] = 0; // no chip found
        } else {
            adc16Installed[i] = 1; // chip found!
            printf("adc_init: ADC board %d found\n", i);
        }
    }
    if (adc16Installed[0] == 0 && adc16Installed[1] == 0) {
        printf("adc_init: No ADC boards found\n");
        printf("check I2C_PORT_SELECTED value, and the two I2C resistors on the board(s)\n");
        return;
    }
}

void rdy_callback(uint gpio, uint32_t events) {
    rdy = true;
}

// board initialisation
void
board_init(void) {
    int i;
    int sanity;
    // I2C init
    if (I2C_PORT_SELECTED == 0) {
        i2c_port = &i2c0_inst;
        printf("board_init: using I2C0 sda=GP%d, scl=GP%d\n", I2C_SDA0_PIN, I2C_SCL0_PIN);
    } else {
        i2c_port = &i2c1_inst;
        printf("board_init: using I2C1 sda=GP%d, scl=GP%d\n", I2C_SDA1_PIN, I2C_SCL1_PIN);
    }
    i2c_init(i2c_port, 1000*1000);// 1 MHz I2C clock
    if (I2C_PORT_SELECTED == 0) {
        gpio_set_function(I2C_SDA0_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL0_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA0_PIN); // weak pull-ups but enable them anyway
        gpio_pull_up(I2C_SCL0_PIN);
    } else {
        gpio_set_function(I2C_SDA1_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL1_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA1_PIN); // weak pull-ups but enable them anyway
        gpio_pull_up(I2C_SCL1_PIN);
    }
    // listener for ADC RDY
    gpio_init(13);
    gpio_pull_up(13);
    gpio_set_dir(13, false);
    gpio_set_irq_enabled_with_callback(13, GPIO_IRQ_EDGE_RISE, true, &rdy_callback);


    // check what ADC boards are installed, and initialize them
    adc_init();


}

void adc_enable_rdy(uint8_t boardnum) {
    uint8_t buf[3];

    // // set to active high
    // buf[0] = ADS1115_REG_CONFIG;
    // adc_confreg[boardnum][1]  |= 0x08; // set bit3 to high for active high 
    // buf[1] = adc_confreg[boardnum][0];
    // buf[2] = adc_confreg[boardnum][1];
    // i2c_write_blocking(i2c_port, adc_addr[boardnum], buf, 3, false);


    buf[0] = ADS1115_REG_LO_THRESH;
    buf[1] = 0x00;
    buf[2] = 0x00; //Lo_thresh MS bit must be 0
    i2c_write_blocking(i2c_port, adc_addr[boardnum], buf, 3, false);
    // i2c_read_blocking(i2c_port, adc_addr[boardnum], buf, 2, false);

    buf[0] = ADS1115_REG_HI_THRESH;
    buf[1] = 0x80;
    buf[2] = 0x00; //Hi_thresh  MS bit must be 1
    i2c_write_blocking(i2c_port, adc_addr[boardnum], buf, 3, false);
    // i2c_read_blocking(i2c_port, adc_addr[boardnum], buf, 2, false);
}

// ************ main function *******************
int main(void) {
    stdio_init_all();

    board_init();

    build_data_rate(BOARD0, DR_860);
    build_gain(BOARD0, AIN1, GAIN_2_048); // +- 2.048V

    #define SAMPLES 860

    uint16_t buf[SAMPLES];
    
    build_cont_conversion(0);
    adc_set_mux(BOARD0,0,false);
    adc_enable_rdy(0);

    uint8_t* buf8_ptr = (uint8_t*)buf;
    *buf8_ptr = ADS1115_REG_CONVERSION;
    i2c_write_blocking(i2c_port, adc_addr[BOARD0], buf8_ptr, 1, false);

    uint8_t* buf8_end_ptr = ((uint8_t*)buf) + (sizeof buf);
    rdy = false;

    while (buf8_ptr < buf8_end_ptr) {
        if (rdy) {
            rdy = false;
            i2c_read_blocking(i2c_port, adc_addr[BOARD0], buf8_ptr, 2, false);
            buf8_ptr += 2;
        }
    }

    uint16_t* buf16_ptr = buf;
    uint16_t* buf16_end_ptr = buf + (sizeof buf) / 2;
    
    while (buf16_ptr < buf16_end_ptr) {
        *buf16_ptr = *buf16_ptr >> 8 | *buf16_ptr << 8; // swap bytes
        printf("AIN1 = %.3f V\n", to_volts(BOARD0, 0, *buf16_ptr));
        buf16_ptr++;
    }

    while (FOREVER) {}
}
