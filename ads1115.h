#ifndef _ADS1115_H_
#define _ADS1115_H_

#include "hardware/gpio.h"
#include "hardware/i2c.h"

class ads1115 {
public:
    enum class addr : uint8_t {
        A_0  = 0x48, 
        A_1,
        A_2,
        A_3,
        A_LAST
    };

    enum class reg : uint8_t {
        REG_CONVERSION = 0x00,
        REG_CONFIG,
        REG_LO_THRESH,
        REG_HI_THRESH
    };

    enum class data_rate : uint8_t {
        DR_8 = 0,
        DR_16,
        DR_32,
        DR_64,
        DR_128,
        DR_250,
        DR_475, 
        DR_860
    };

    enum class gain : uint8_t {
        GAIN_6_144 = 0,
        GAIN_4_096,
        GAIN_2_048,
        GAIN_1_024,
        GAIN_0_512,
        GAIN_0_256
    };

    enum class channel : uint8_t {
        CH_AIN1 = 0,
        CH_AIN2
    };

    enum class diff: uint8_t {
        DIFF_0_1 = 0x00,
        DIFF_2_3 = 0x03
    };

private:
    volatile bool data_ready;
    i2c_inst_t *i2c_port;
    addr address;
    data_rate dr;
    uint8_t adc_confreg[2] = {0, 0};
    gain adc_gain_bits[2] = {gain::GAIN_2_048, gain::GAIN_2_048};
    double adc_range[2] = {2.048, 2.048};
    double adc_range_value[8] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
    double opamp_gain[2] = {0.38298, 0.38298};

    // safe conversions, only available within the class implementation
    inline uint8_t operator()(addr a) const {return static_cast<uint32_t>(a);}
    inline addr operator()(uint8_t a) const {
        assert(a >= (*this)(addr::A_0) && a < (*this)(addr::A_LAST)); // debug support
        return  static_cast<addr>(a);
    }
    inline uint8_t operator()(reg r) const {return static_cast<uint32_t>(r);}
    inline uint8_t operator()(data_rate dr) const {return static_cast<uint32_t>(dr);}
    inline uint8_t operator()(gain g) const {return static_cast<uint32_t>(g);}
    inline uint8_t operator()(channel c) const {return static_cast<uint32_t>(c);}
    inline uint8_t operator()(diff d) const {return static_cast<uint32_t>(d);}

public:
    ads1115() : data_ready(false), i2c_port(nullptr), address(addr::A_LAST), dr(data_rate::DR_860) {}

    bool init(i2c_inst_t *i2c_port); // configure for first found IC
    bool init(i2c_inst_t *i2c_port, addr address); // configure for IC with provided address

    inline bool is_data_ready() {return data_ready;}
    inline void set_data_ready(bool data_ready) {this->data_ready = data_ready;}

    void build_data_rate(data_rate dr);
    void build_gain(channel chan, gain gain);
    void build_cont_conversion();
    void build_single_conversion();

    void adc_set_mux(channel chan, bool do_single_conversion);
    void adc_enable_ready();

    void bulk_read(uint16_t* buf, size_t len);

    void start_single_conversion();
    uint16_t adc_raw_diff_result();

    // convert the raw value to volts present at the ADC input
    // adjust for opamp gain
    inline double to_volts(channel chan, uint16_t raw) {
        return (double)(((int16_t)raw) * (adc_range[(uint8_t)chan] / 32768.0)) / opamp_gain[(uint8_t)chan];
    }
};

#endif // _ADS1115_H_
