cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 23)

include(pico_sdk_import.cmake)
# uncomment the following line if using a Pico W board
set(PICO_BOARD pico_w)
# set the project name (change to suit)
project(adc_board_test)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "  Diag: maximum debug info")
    #I've set this to allow breakpoints on any source line
    set(PICO_DEOPTIMIZED_DEBUG=1)
    add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function 
        -Wno-maybe-uninitialized
        -g3
    )
endif ()


pico_sdk_init()

# add_subdirectory()

# rest of your project
add_executable(${CMAKE_PROJECT_NAME}
        main.cpp
        ads1115.cpp
        )

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(${CMAKE_PROJECT_NAME} 
        pico_stdlib
        hardware_i2c
        hardware_irq
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
