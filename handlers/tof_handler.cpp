#include "tof_handler.hpp"

void TaskUpdate(void *pvParameters)
{
    // Retrieve the TOF10120 sensor object passed as a parameter
    TOF10120* tof_sensor = static_cast<TOF10120*>(pvParameters);

    while (true)
    {
        // Get the distance from the sensor
        uint16_t distance = tof_sensor->get_distance();
        uint16_t filtered_distance = tof_sensor->apply_sma(&distance);

        // Create a reading structure
        tof_data_t reading;
        reading.distance = filtered_distance;
        reading.xTimeStamp = xTaskGetTickCount();

        // Send the reading to the mailbox
        if (xQueueSend(xMailbox, &reading, pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("Mailbox full. Timestamp: %u, Distance: %u mm\n",
                   reading.xTimeStamp, reading.distance);
        }

        // Add a delay to prevent excessive polling
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task to process the ToF readings (consumer task)
void TaskRead(void *pvParameters)
{
    tof_data_t reading;

    while (true)
    {
        // Wait for a new reading to be available in the mailbox
        if (xQueueReceive(xMailbox, &reading, portMAX_DELAY) == pdPASS)
        {
            printf("Distance: %u mm, Timestamp: %u\n", reading.distance, reading.xTimeStamp);
        }
    }
}
// End of file
