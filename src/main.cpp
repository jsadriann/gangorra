#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "mpu6050_handler.h"

QueueHandle_t sensorDataQueue;

int main() {
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    mpu6050_init();

    sensorDataQueue = xQueueCreate(10, sizeof(int16_t) * 6);

    create_sensor_tasks(sensorDataQueue);

    vTaskStartScheduler();

    while (1) {}
}
