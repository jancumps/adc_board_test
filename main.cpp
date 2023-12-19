/************************************************************************
 * adc_board_test
 * main.cpp
 * rev 1.0 - shabaz - oct 2023
 * convert to bulk mode and cpp jc 20231218
 ************************************************************************/

// ************* header files ******************

#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

#include "ads1115.h"

#include <iterator>
#include <numeric>

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

#define ADC_RDY 13

// sample batch size in bulk mode
#define SAMPLES 860

// ************ global variables ****************

i2c_inst_t *i2c_port;
ads1115 ads = ads1115();

// ********** functions *************************

void rdy_callback(uint gpio, uint32_t events) {
    switch (gpio) {
        case ADC_RDY:
            ads.set_data_ready(true);
            break;
        default:
            break;
    }
}

// board initialisation
void board_init() {
    // I2C init
    if (I2C_PORT_SELECTED == 0) {
        i2c_port = &i2c0_inst;
    } else {
        i2c_port = &i2c1_inst;
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
 
    // trigger and listener for ADC RDY pin
    gpio_init(ADC_RDY);
    gpio_pull_up(ADC_RDY);
    gpio_set_dir(ADC_RDY, false);
    gpio_set_irq_enabled_with_callback(ADC_RDY, GPIO_IRQ_EDGE_RISE, true, &rdy_callback);
}

// ************ main function *******************
int main(void) {

    stdio_init_all();
    board_init();

    // check what ADC boards are installed, and initialize them
    ads.init(i2c_port);
    sleep_ms(5 * 1000);

    ads.build_data_rate(ads1115::data_rate::DR_860);
    ads.build_gain(ads1115::channel::CH_AIN1, ads1115::gain::GAIN_2_048); // +- 2.048V

    
    ads.build_cont_conversion();
    ads.adc_set_mux(ads1115::channel::CH_AIN1, false);

    uint16_t buf[SAMPLES];

    ads.adc_enable_ready();
    ads.set_data_ready(false);
    ads.bulk_read(std::begin(buf), std::end(buf));

    // buffer bytes will now get swapped, 
    // and that's needed for the next actions
    // use stl iterator and lambda function
    std::for_each(std::begin(buf), std::end(buf), [](uint16_t& u){ u = u >> 8 | u << 8; }); // swap bytes

    // statistics. Prereq: bytes are already swapped

    // average
    uint16_t average = std::accumulate(std::begin(buf), std::end(buf), 0.0) / std::size(buf);
    printf("AVG = %.7f V\n", ads.to_volts(ads1115::channel::CH_AIN1, average));
    // min and max
    auto minmax = std::minmax_element(std::begin(buf), std::end(buf));
    printf("MIN = %.7f V\n", ads.to_volts(ads1115::channel::CH_AIN1, *minmax.first));
    printf("MAX = %.7f V\n", ads.to_volts(ads1115::channel::CH_AIN1, *minmax.second));

    // convert every value to voltage
    double volts[SAMPLES];
    std::transform(std::begin(buf), std::end(buf), std::begin(volts),
               [](uint16_t u){ return ads.to_volts(ads1115::channel::CH_AIN1, u); });

    // print voltage
    std::for_each(std::begin(volts), std::end(volts), [](double& d) { 
        printf("SMP = %.7f V\n", d);
    }); 

    while (true) {        
    }
}
