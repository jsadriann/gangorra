#include "i2c_sensors.hpp"

i2c_sensors::~i2c_sensors()
{
        printf("I2C interface deinitialized\n");
        i2c_deinit(i2c_port);
}

void i2c_sensors::initialize() const
{
        i2c_init(i2c_port, baudrate);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
}
// End of file

