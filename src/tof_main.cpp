#include "i2c_sensors.hpp"
#include "tof_handler.hpp"
#include "tof.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define QUEUE_SIZE 1

__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

QueueHandle_t xMailbox = NULL;

int main()
{
    stdio_init_all();               // Starting selected serial port
    
    // Create the mailbox to store ToF readings
    xMailbox = xQueueCreate(QUEUE_SIZE, sizeof(tof_data_t));
    if (xMailbox == NULL)
    {
        printf("Failed to create mailbox\n");
        return -1;
    }

    // Create sensor object with specific I2C pins and address
    const uint sda = 16;
    const uint scl = 17;
    const uint8_t addr = TOF_ADDR;
    const uint32_t baudrate = 400000;

    // Instantiate the TOF10120 sensor
    TOF10120 tof_sensor(i2c0, sda, scl, addr, baudrate);

    // Initialize the sensor
    tof_sensor.initialize();
    
    // Create the ToF producer task, passing the sensor object as a parameter
    xTaskCreate(TaskUpdate, 
		    "Send data to mailbox", 
		    1024,
		    &tof_sensor,
		    1,
		    NULL);

    // Create the ToF consumer task
    xTaskCreate(TaskRead,
		    "Retrieve data from mailbox",
		    1024,
		    NULL,
		    1,
		    NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // Code should never reach here
    while (true) {}

    return 0;
}
// End of file

