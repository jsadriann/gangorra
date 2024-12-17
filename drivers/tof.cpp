// TO-DO: Test
#include "tof.hpp"

uint16_t TOF10120::get_distance()
{
        uint8_t buffer[2] = {0}; // Buffer to store raw sensor data
        uint8_t write_addr = 0x00; // Distance register address

        i2c_write_blocking(i2c_port, device_addr, &write_addr, 1, true); // Point to write register
        i2c_read_blocking(i2c_port, device_addr, buffer, 2, false);  // Read 2 bytes

        return (buffer[0] << 8) | buffer[1]; // Combine bytes to form 16-bit distance
}

void TOF10120::to_string(uint16_t *distance)
{
        printf("Distance (mm): %d\n", *distance);
}
// End of file

