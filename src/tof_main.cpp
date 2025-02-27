#include "i2c_sensors.hpp"
#include "tof_handler.hpp"
#include "tof.hpp"
#include "VL53L0X.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define QUEUE_SIZE 1

__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

QueueHandle_t xTOFMailbox = NULL;
QueueHandle_t xVLMailbox = NULL;

int main()
{
    stdio_init_all();               // Starting selected serial port
    
    xTOFMailbox = xQueueCreate(QUEUE_SIZE, sizeof(tof10120_data_t));
    xVLMailbox = xQueueCreate(QUEUE_SIZE, sizeof(vl53l0x_data_t));

    if (xTOFMailbox == NULL || xVLMailbox == NULL) {
        printf("Failed to create mailboxes\n");
        return -1;
    }

    // Create sensor objects with specific I2C pins and address
    const uint sda0 = 12, scl0 = 13;  // TOF10120
    const uint sda1 = 14, scl1 = 15;  // VL53L0X
    const uint32_t baudrate = 400000;
    
    // Instantiate TOF10120 sensor
    const uint8_t tof_addr = TOF_ADDR;
    TOF10120 tof_sensor(i2c0, sda0, scl0, tof_addr, baudrate);
    tof_sensor.initialize();
    
    // Instantiate VL53L0X sensor
    const uint8_t vl_addr = VL53L0X_ADDR;
    VL53L0X vl_sensor;
    if (!vl_sensor.init()) {
        printf("VL53L0X initialization failed!\n");
        return -1;
    }
    vl_sensor.setAddress(vl_addr);
    vl_sensor.startContinuous();

    // Create ToF producer task for both sensors
    xTaskCreate(TaskUpdate, "Send TOF data", 1024, &tof_sensor, 1, NULL);
    xTaskCreate(TaskVLUpdate, "Send VL data", 1024, &vl_sensor, 1, NULL);

    // Create the ToF consumer task
    xTaskCreate(TaskRead, "Retrieve data", 1024, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    while (true) {}
    return 0;
}
// End of file

