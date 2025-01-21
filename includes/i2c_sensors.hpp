#ifndef I2C_SENSORS_HPP
#define I2C_SENSORS_HPP

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Superclass for general I2C setup
class i2c_sensors
{
protected:
    i2c_inst_t* i2c_port;
    uint sda_pin;
    uint scl_pin;
    uint32_t baudrate = 400000;
public:
    // Constructor
    i2c_sensors(i2c_inst_t* i2c, uint sda, uint scl, uint32_t baudrate)
        : i2c_port(i2c), sda_pin(sda), scl_pin(scl), baudrate(baudrate){}

    // Destructor
    virtual ~i2c_sensors();

    // Initialize I2C
    void initialize() const;
};

#endif  // I2C_SENSORS_HPP

