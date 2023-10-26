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
#define I2C_PORT_SELECTED 1
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

// EEPROM (CAT24AA01) definitions
// If two ADC boards are installed then one EEPROM needs to be removed, or replaced with
// a different EEPROM with a different address. For now, one EEPROM would need to store the
// calibration data for both ADC boards.
#define EEPROM_I2C_ADDR 0x50
#define EEPROM_PAGE_SIZE 16
#define EEPROM_PAGE_COUNT 8
// EEPROM content locations, used for configuration data
#define CONTENT_BYTE 0
#define CONTENT_FLOAT 1
#define CONTENT_DOUBLE 2
// 4 bytes for any magic and identifier
#define LOC_ID 0
// 0V offsets (float values. 4 bytes each)
#define LOC_BOARD0_AIN1_OFFSET 4
#define LOC_BOARD0_AIN2_OFFSET 8
#define LOC_BOARD1_AIN1_OFFSET 12
#define LOC_BOARD1_AIN2_OFFSET 16
// ADC gain settings (1 byte each)
#define LOC_BOARD0_AIN1_GAIN 20
#define LOC_BOARD0_AIN2_GAIN 21
#define LOC_BOARD1_AIN1_GAIN 22
#define LOC_BOARD1_AIN2_GAIN 23
// op-amp gain settings (float values. 4 bytes each)
#define LOC_BOARD0_AIN1_OPAMP_GAIN 24
#define LOC_BOARD0_AIN2_OPAMP_GAIN 28
#define LOC_BOARD1_AIN1_OPAMP_GAIN 32
#define LOC_BOARD1_AIN2_OPAMP_GAIN 36
// number of items that will appear in the configuration menu
#define NUM_CONFIG_ITEMS 12



// Temperature sensor (STS40) definitions
// If two ADC boards are installed then one temperature sensor needs to be removed.

// misc
#define FOREVER 1


// ************ constants **********************
uint8_t adc_addr[2] = {ADC_ADDR0, ADC_ADDR1};
uint32_t conv_time_ms[8] = {125, 63, 32, 16, 8, 4, 3, 2}; // conversion time in us
double adc_range_value[8] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
// EEPROM configuration data
char cfg_text[NUM_CONFIG_ITEMS][48] = {"Board #0 0V Offset for AIN1 (V)",
                         "Board #0 0V Offset for AIN2 (V)",
                         "Board #1 0V Offset for AIN1 (V)",
                         "Board #1 0V Offset for AIN2 (V)",
                         "Board #0 Gain code for AIN1 (0-7)",
                         "Board #0 Gain code for AIN2 (0-7)",
                         "Board #1 Gain code for AIN1 (0-7)",
                         "Board #1 Gain code for AIN2 (0-7)",
                         "Board #0 Opamp Gain value for AIN1",
                         "Board #0 Opamp Gain value for AIN2",
                         "Board #1 Opamp Gain value for AIN1",
                         "Board #1 Opamp Gain value for AIN2"};
uint8_t cfg_type[NUM_CONFIG_ITEMS] = {CONTENT_FLOAT, CONTENT_FLOAT, CONTENT_FLOAT, CONTENT_FLOAT,
                        CONTENT_BYTE, CONTENT_BYTE, CONTENT_BYTE, CONTENT_BYTE,
                        CONTENT_FLOAT, CONTENT_FLOAT, CONTENT_FLOAT, CONTENT_FLOAT};
uint16_t cfg_loc[NUM_CONFIG_ITEMS]= {LOC_BOARD0_AIN1_OFFSET, LOC_BOARD0_AIN2_OFFSET, LOC_BOARD1_AIN1_OFFSET, LOC_BOARD1_AIN2_OFFSET,
                       LOC_BOARD0_AIN1_GAIN, LOC_BOARD0_AIN2_GAIN, LOC_BOARD1_AIN1_GAIN, LOC_BOARD1_AIN2_GAIN,
                       LOC_BOARD0_AIN1_OPAMP_GAIN, LOC_BOARD0_AIN2_OPAMP_GAIN, LOC_BOARD1_AIN1_OPAMP_GAIN, LOC_BOARD1_AIN2_OPAMP_GAIN};


// ************ global variables ****************
i2c_inst_t *i2c_port;
uint8_t adc16Installed[2];
uint8_t adc_confreg[2][2]; // config register for ADC
uint8_t adc_data_rate[2] = {DR_128, DR_128};
uint8_t adc_gain_bits[2] = {GAIN_2_048, GAIN_2_048}; // +/- 2.048V
double adc_range[2] = {2.048, 2.048};
double opamp_gain[2] = {0.38298, 0.38298}; // depends on opamp gain resistor values


// ************ function prototypes *************
void adc_init(void); // initializes both ADC boards if present
void build_cont_conversion(uint8_t boardnum); // sets up the local config for continuous conversion
void build_single_conversion(uint8_t boardnum); // sets up the local config for single-shot conversion
void build_data_rate(uint8_t boardnum, uint8_t dr); // sets up the local config for data rate (e.g. DR_128)
void build_gain(uint8_t boardnum, uint8_t gain); // sets up the local config for gain (e.g. GAIN_2_048)
// adc_set_mux: programs the config and sets the channel (e.g. AIN1) and also starts the conversion
// if in continuous mode. Note: first result after changing mux in continuous mode will be invalid
// returns 0 if the ADC board was not installed. If do_single_conversion is 1, then the ADC is
// set to single-shot mode, and the conversion is started.
int adc_set_mux(uint8_t boardnum, uint8_t chan, uint8_t do_single_conversion);
int16_t adc_raw_diff_result(uint8_t boardnum); // returns the raw differential result (16-bit signed integer)
double to_volts(uint8_t boardnum, int16_t raw); // converts the raw result to volts
// EEPROM functions: dev_addr is the I2C address 0x50 (EEPROM_I2C_ADDR) for CAT24AA01
// mem_addr is the EEPROM memory address (0-127).
// page_addr is the EEPROM page address (0-7), corresponding to the 8 pages of 16 bytes each.
void eeprom_byte_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t data); // writes a byte to EEPROM
// recommend writing doubles at addresses 0, 8, 16, 24, 32, etc., so that they do not straddle pages
void eeprom_write_double(uint8_t dev_addr, uint8_t mem_addr, double data); // writes a double to EEPROM
void eeprom_write_float(uint8_t dev_addr, uint8_t mem_addr, float data); // writes a float to EEPROM
// data_len should be 16 or less.
void eeprom_page_write(uint8_t dev_addr, uint8_t page_num, uint8_t* data, uint8_t datalen);
uint8_t eeprom_byte_read(uint8_t dev_addr, uint8_t mem_addr); // reads a single byte from EEPROM
double eeprom_read_double(uint8_t dev_addr, uint8_t mem_addr); // reads a double from EEPROM
float eeprom_read_float(uint8_t dev_addr, uint8_t mem_addr); // reads a float from EEPROM
void eeprom_sequential_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t* data, uint8_t datalen);
void startup_button_action(uint8_t startup_button_state); // used to quickly store 0V offsets in EEPROM

// ********** functions *************************
void
print_title(void) {
    printf("\n\n");
    printf("Project built on %s %s\n", __DATE__, __TIME__);
    printf("\n");
}

// general-purpose long delay timer if required
void
sleep_sec(uint32_t s) {
    sleep_ms(s * 1000);
}

// Configuration menu related functions
void print_cfg_menu(void) {
    int i;
    printf("\nConfiguration Menu:\n");
    for (i=0; i<NUM_CONFIG_ITEMS; i++) {
        printf("(%c) %s ", i+'a', cfg_text[i]);
        if (cfg_type[i] == CONTENT_BYTE) {
            printf("[%d]\n", eeprom_byte_read(EEPROM_I2C_ADDR, cfg_loc[i]));
        } else if (cfg_type[i] == CONTENT_FLOAT) {
            printf("[%f]\n", eeprom_read_float(EEPROM_I2C_ADDR, cfg_loc[i]));
        } else if (cfg_type[i] == CONTENT_DOUBLE) {
            printf("[%f]\n", eeprom_read_double(EEPROM_I2C_ADDR, cfg_loc[i]));
        } else {
            printf("\nprint_cfg_menu internal error, unknown type\n");
        }
    }
    printf("press a key (a-%c) or 'x' to exit: ", 'a'+NUM_CONFIG_ITEMS-1);
}
char check_for_keypress(void) {
    int c;
    c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;
    } else {
        return c;
    }
}
// wait for a user entry of a string
int wait_for_string_entry(char* buf, int buflen) {
    int retval = 0;
    int i = 0;
    int not_finished = 1;
    while (not_finished) {
        char c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
        // handle return(enter) kepress
        if (c == 0x0d) {
            printf("\n");
            buf[i] = 0;
            not_finished = 0;
        }
        // handle backspace (BS) control character
        if (c == 0x08) {
            if (i > 0) {
                printf("\b \b");
                i--;
            }
            continue;
        }
        //handle all printable characters
        if (c>=32 && c<=126) {
            if (i < buflen - 1) {
                printf("%c", c);
                buf[i++] = c;
            }
        }
    }
    if (retval == 0) {
        return(i); // returns the string length
    } else {
        return(retval); // return any error value (negative)
    }
}
uint8_t wait_byte_entry(void) {
    char buf[4];
    int not_finished = 1;
    int len;
    int v = 0;
    while(not_finished) {
        len = wait_for_string_entry(buf, 4);
        if (len == 0) {
            continue;
        } else {
            sscanf(buf, "%d", &v);
            if (v < 0 || v > 255) {
                printf("Invalid or out of range value, try again: ");
                continue;
            } else {
                not_finished = 0;
            }
        }
    }
    return((uint8_t)v);
}
double wait_double_entry(void) {
    char buf[16];
    int not_finished = 1;
    int len;
    double v = 0.0;
    while(not_finished) {
        len = wait_for_string_entry(buf, 16);
        if (len == 0) {
            continue;
        } else {
            sscanf(buf, "%lf", &v);
            not_finished = 0;
        }
    }
    return(v);
}
float wait_float_entry(void) {
    char buf[16];
    int not_finished = 1;
    int len;
    float v = 0.0;
    while(not_finished) {
        len = wait_for_string_entry(buf, 16);
        if (len == 0) {
            continue;
        } else {
            sscanf(buf, "%f", &v);
            not_finished = 0;
        }
    }
    return(v);
}
// this function prints the config menu and handles all the user input,
// programming the EEPROM as required.
void cfg_menu(void) {
    int not_finished = 1;
    int i;
    uint8_t v_byte, old_byte;
    double v_double, old_double;
    float v_float, old_float;
    print_cfg_menu();
    while (not_finished) {
        char c = check_for_keypress();
        if (c) {
            // OK a key was pressed. Handle it.
            printf("%c\n", c);
            printf("Enter new value: ");
            if (c == 'x') {
                printf("Exiting config menu\n");
                not_finished = 0;
            } else if ((c==' ') | (c=='\n') | (c=='\r')) {
                // re-print the menu
                print_cfg_menu();
            } else {
                // was the selection invalid?
                i = c - 'a';
                if (i < 0 || i >= NUM_CONFIG_ITEMS) {
                    printf("Invalid selection, try again: ");
                    continue;
                }
                // a menu item was definitely selected
                switch(cfg_type[i]) {
                    case CONTENT_BYTE:
                        v_byte = wait_byte_entry();
                        old_byte = eeprom_byte_read(EEPROM_I2C_ADDR, cfg_loc[i]);
                        printf("Previously stored value is %d\n", old_byte);
                        if (v_byte != old_byte) {
                            printf("Replacing with %d\n", v_byte);
                            eeprom_byte_write(EEPROM_I2C_ADDR, cfg_loc[i], v_byte);
                            printf("Done.\n");
                        } else {
                            printf("No change required.\n");
                        }
                        break;
                    case CONTENT_DOUBLE:
                        v_double = wait_double_entry();
                        old_double = eeprom_read_double(EEPROM_I2C_ADDR, cfg_loc[i]);
                        printf("Previously stored value is %lf\n", old_double);
                        if (v_double != old_double) {
                            printf("Replacing with %lf\n", v_double);
                            eeprom_write_double(EEPROM_I2C_ADDR, cfg_loc[i], v_double);
                            printf("Done.\n");
                        } else {
                            printf("No change required.\n");
                        }
                        break;
                    case CONTENT_FLOAT:
                        v_float = wait_float_entry();
                        old_float = eeprom_read_float(EEPROM_I2C_ADDR, cfg_loc[i]);
                        printf("Previously stored value is %f\n", old_float);
                        if (v_float != old_float) {
                            printf("Replacing with %f\n", v_float);
                            eeprom_write_float(EEPROM_I2C_ADDR, cfg_loc[i], v_float);
                            printf("Done.\n");
                        } else {
                            printf("No change required.\n");
                        }
                        break;
                    default:
                        printf("Internal error on cfg_type!\n");
                        break;
                }
            }
        } // keep checking for keypress
    } // end while(not_finished)
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
void build_gain(uint8_t boardnum, uint8_t gain) {
    if (gain > 5) {
        printf("build_gain: invalid gain\n");
        return;
    }
    adc_range[boardnum] = adc_range_value[gain];
    adc_gain_bits[boardnum] = gain;
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

double to_volts(uint8_t boardnum, int16_t raw) {
    double v;
    v = (double)raw * (adc_range[boardnum] / 32768.0); // convert the raw value to volts present at the ADC input
    v = v / opamp_gain[boardnum]; // adjust for opamp gain
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
        i2c_read_blocking(i2c_port, adc_addr[i], buf, 2, false);
        if (buf[0] != 0x85 || buf[1] != 0x83) {
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

void eeprom_byte_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t data) {
    uint8_t buf[2];
    buf[0] = mem_addr;
    buf[1] = data;
    i2c_write_blocking(i2c_port, dev_addr, buf, 2, false);
    sleep_ms(6); // wait at least 5 msec for write to complete
}

void eeprom_write_double(uint8_t dev_addr, uint8_t mem_addr, double data) {
    uint8_t buf[9];
    uint8_t* buf8_ptr = (uint8_t*)&data;
    buf[0] = mem_addr;
    memcpy(buf+1, buf8_ptr, 8);
    i2c_write_blocking(i2c_port, dev_addr, buf, 9, false);
    sleep_ms(6); // wait at least 5 msec for write to complete
}

void eeprom_write_float(uint8_t dev_addr, uint8_t mem_addr, float data) {
    uint8_t buf[5];
    uint8_t* buf8_ptr = (uint8_t*)&data;
    buf[0] = mem_addr;
    memcpy(buf+1, buf8_ptr, 4);
    i2c_write_blocking(i2c_port, dev_addr, buf, 5, false);
    sleep_ms(6); // wait at least 5 msec for write to complete
}

void eeprom_page_write(uint8_t dev_addr, uint8_t page_num, uint8_t* data, uint8_t datalen) {
    uint8_t buf[17];
    buf[0] = page_num * EEPROM_PAGE_SIZE;
    memcpy(buf+1, data, datalen);
    i2c_write_blocking(i2c_port, dev_addr, buf, datalen+1, false);
    sleep_ms(6); // wait at least 5 msec for write to complete
}

uint8_t eeprom_byte_read(uint8_t dev_addr, uint8_t mem_addr) {
    uint8_t buf[1];
    buf[0] = mem_addr;
    i2c_write_blocking(i2c_port, dev_addr, buf, 1, true);
    i2c_read_blocking(i2c_port, dev_addr, buf, 1, false);
    return(buf[0]);
}

double eeprom_read_double(uint8_t dev_addr, uint8_t mem_addr) {
    uint8_t buf[8];
    uint8_t* buf8_ptr = (uint8_t*)&buf;
    buf[0] = mem_addr;
    i2c_write_blocking(i2c_port, dev_addr, buf, 1, true);
    i2c_read_blocking(i2c_port, dev_addr, buf, 8, false);
    return(*(double*)buf8_ptr);
}

float eeprom_read_float(uint8_t dev_addr, uint8_t mem_addr) {
    uint8_t buf[4];
    uint8_t* buf8_ptr = (uint8_t*)&buf;
    buf[0] = mem_addr;
    i2c_write_blocking(i2c_port, dev_addr, buf, 1, true);
    i2c_read_blocking(i2c_port, dev_addr, buf, 4, false);
    return(*(float*)buf8_ptr);
}

void eeprom_sequential_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t* data, uint8_t datalen) {
    uint8_t buf[1];
    buf[0] = mem_addr;
    i2c_write_blocking(i2c_port, dev_addr, buf, 1, true);
    i2c_read_blocking(i2c_port, dev_addr, data, datalen, false);
}


void startup_button_action(uint8_t startup_button_state) {
    int chan_index;
    int16_t raw[2][2];
    double v[2][2];
    // handle any button that was pressed at startup
    switch(startup_button_state) {
        case 1: // Button on Pico-Eurocard was pressed
        case 2: // Y button on the display board was pressed
            // launch the configuration menu
            cfg_menu();
            break;
        default:
            break;
    }
}

// board initialisation
void
board_init(void) {
    // I2C init
    if (I2C_PORT_SELECTED == 0) {
        i2c_port = &i2c0_inst;
        printf("board_init: using I2C0 sda=GP%d, scl=GP%d\n", I2C_SDA0_PIN, I2C_SCL0_PIN);
    } else {
        i2c_port = &i2c1_inst;
        printf("board_init: using I2C1 sda=GP%d, scl=GP%d\n", I2C_SDA1_PIN, I2C_SCL1_PIN);
    }
    i2c_init(i2c_port, 1E5);// 100kHz I2C clock
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

    // initialize the ADC board(s)
    adc_init();

}


// ************ main function *******************
int main(void) {
    int startup_button_state = 0;
    int chan_index;
    int board_index;
    int16_t raw[2][2];
    double v[2][2];
    stdio_init_all();
    // check to see if a certain button was pressed on startup
    // we need to configure any buttons needed to do that.
    gpio_init(BTN_EURO);
    gpio_set_dir(BTN_EURO, GPIO_IN);
    gpio_set_pulls(BTN_EURO, true, false); // pullup enabled
    gpio_init(BTN_DISP_Y);
    gpio_set_dir(BTN_DISP_Y, GPIO_IN);
    gpio_set_pulls(BTN_DISP_Y, true, false); // pullup enabled
    if (PRESSED(BTN_EURO)) {
        startup_button_state = 1;
    } else if (PRESSED(BTN_DISP_Y)) {
        startup_button_state = 2;
    }
    sleep_ms(3000); // could remove this after debugging, or keep it in

    print_title(); // print welcome on USB UART or Serial UART (selected in CMakelists.txt)
    board_init();

    // set up the variables for the desired gain and data rate for both boards
    // these will be sent to the ADC board when the channel is selected
    // (using adc_set_mux)
    for (board_index = 0; board_index < 2; board_index++) {
        build_data_rate(board_index, DR_128);
        build_gain(board_index, GAIN_2_048); // +- 2.048V
    }

    startup_button_action(startup_button_state); // action any button that was pressed at startup


    while (FOREVER) {
        // read AIN1 for both boards, then read AIN2 for both boards:
        for (chan_index = 0; chan_index < 2; chan_index++) {
            // select the channel for the conversions
            adc_set_mux(BOARD0, chan_index, DO_SINGLE_CONVERSION);
            adc_set_mux(BOARD1, chan_index, DO_SINGLE_CONVERSION);
            // wait for conversion to complete on both boards
            // assume for now that both boards are using the same data rate!
            sleep_ms(conv_time_ms[adc_data_rate[BOARD0]]);
            sleep_ms(5); // wait a little longer for margin
            raw[BOARD0][chan_index] = adc_raw_diff_result(BOARD0); // read the conversion result for board 0
            raw[BOARD1][chan_index] = adc_raw_diff_result(BOARD1); // read the conversion result for board 1
        }
        // ok now we have up to 4 raw values, convert them to volts
        for (board_index = 0; board_index < 2; board_index++) {
            if (adc16Installed[board_index]) {
                v[board_index][AIN1] = to_volts(board_index, raw[board_index][AIN1]);
                v[board_index][AIN2] = to_volts(board_index, raw[board_index][AIN2]);
                printf("Board%d: AIN1 = %.3f V, AIN2 = %.3f V\n", board_index, v[board_index][AIN1], v[board_index][AIN2]);
            }
        }
        sleep_ms(500);
    } // end while(FOREVER)
}
