#include "tof.hpp"

#define SMA_WINDOW_SIZE 5  // Adjustable based on the desired smoothing level

// Circular buffer for the moving average
uint16_t buffer[SMA_WINDOW_SIZE] = {0};
uint16_t sum = 0;
uint8_t ind = 0;
uint8_t count = 0;

uint16_t TOF10120::get_distance()
{
	uint8_t buffer[2] = {0}; // Buffer to store raw sensor data
        uint8_t write_addr = 0x00; // Distance register address

        i2c_write_blocking(i2c_port, device_addr, &write_addr, 1, true); // Point to write register
        i2c_read_blocking(i2c_port, device_addr, buffer, 2, false);  // Read 2 bytes

        return (buffer[0] << 8) | buffer[1]; // Combine bytes to form 16-bit distance
}

uint16_t TOF10120::apply_sma(uint16_t *distance)
{
    // Subtract the oldest value from the sum (only if buffer is full)
    if (count >= SMA_WINDOW_SIZE)
    {
        sum -= buffer[ind];
    }
    else
    {
        count++;
    }

    // Add the new measurement to the sum and the buffer
    buffer[ind] = *distance;
    sum += *distance;

    // Advance the circular buffer index
    ind = (ind + 1) % SMA_WINDOW_SIZE;

    // Return the average
    return (sum / count);
}
// End of file
