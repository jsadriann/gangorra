#ifndef TOF_HPP
#define TOF_HPP

#include "i2c_sensors.hpp"

static const uint8_t TOF_ADDR = 0x52;

// Subclass for TOF10120 sensor
class TOF10120 : public i2c_sensors
{
private:
    uint8_t device_addr;

public:
    // Constructor
    TOF10120(i2c_inst_t* i2c, uint sda, uint scl, uint8_t addr, uint32_t baudrate)
        : i2c_sensors(i2c, sda, scl, baudrate), device_addr(addr){}

    // Read distance measurement
    uint16_t get_distance();

    // Display distance measurement
    void to_string(uint16_t *distance);
};

#endif // TOF_HPP
