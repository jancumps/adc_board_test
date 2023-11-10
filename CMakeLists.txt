cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
# uncomment the following line if using a Pico W board
set(PICO_BOARD pico_w)
# set the project name (change to suit)
project(adc_board_test)

pico_sdk_init()

# add_subdirectory()

# rest of your project
add_executable(${CMAKE_PROJECT_NAME}
        main.cpp
        )

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        )

target_link_libraries(${CMAKE_PROJECT_NAME} 
        pico_stdlib
        hardware_i2c
        )

# if environment variable I2C_PORT_SELECTED declared, pass it on to the compiler
if(DEFINED ENV{I2C_PORT_SELECTED})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE I2C_PORT_SELECTED=$ENV{I2C_PORT_SELECTED})
endif()

# adjust to enable stdio via usb, or uart
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
