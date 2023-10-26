# adc_board_test

Board bring-up/test program for ADS1115-based ADC board.

This test program can be used to verify the hardware, and to prototype or try out features. The code here is not production-grade, and resides in a single large main.cpp file, it should be broken out into separate files for a real application. Configuration values are not (significantly) sanity-checked. Use this repo to quickly try out things and to prototype code that could be copy-pasted either whole or in fragments into your application.

## Current functionality
(a) Prints out voltage values to console every second

(b) Hold down a button wired to GP27 or GP21 (either is fine) at start-up, to go into a configuration menu mode

(c) In configuration menu mode, press Enter in the serial console, to see a configuration menu. Values are written to EEPROM.

## To do
(a) Read the configuration from the EEPROM and use it

(b) Read temperature sensor and print the value to the console

(c) Allow plugging on a LCD screen for displaying the voltage values without requiring a serial console connection. A suitable display is [Waveshare Pico-LCD-1.3](https://www.waveshare.com/pico-lcd-1.3.htm) since it can plug on top, and could be suitable for an end application (e.g. display current status while measurements are logged/plotted on a PC).

