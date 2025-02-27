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
        tof10120_data_t reading;
        reading.distance = filtered_distance;
        reading.xTimeStamp = xTaskGetTickCount();

        // Send the reading to the mailbox
        if (xQueueSend(xTOFMailbox, &reading, pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("Mailbox full. Timestamp: %u, Distance: %u mm\n",
                   reading.xTimeStamp, reading.distance);
        }

        // Add a delay to prevent excessive polling
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void TaskVLUpdate(void *pvParameters)
{
    // Retrieve the VL53L0X sensor object passed as a parameter
    VL53L0X* vl_sensor = static_cast<VL53L0X*>(pvParameters);

    while (true)
    {
        // Get the distance from the VL53L0X sensor
        uint16_t distance = vl_sensor->readRangeContinuousMillimeters();
        
        // Create a reading structure
        vl53l0x_data_t reading;
        reading.distance = distance;
        reading.xTimeStamp = xTaskGetTickCount();

        // Send the reading to the mailbox
        if (xQueueSend(xVLMailbox, &reading, pdMS_TO_TICKS(100)) != pdPASS)
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
    tof10120_data_t tof_reading;
    vl53l0x_data_t vl_reading;

    while (true)
    {
        // Data read in the queues of tof10120 and vl53l0x
        if (xQueueReceive(xTOFMailbox, &tof_reading, portMAX_DELAY) == pdPASS)
        {
            printf("Timestamp: %u, TOF10120: %u mm\n", 
                   tof_reading.xTimeStamp, tof_reading.distance);
        }

        if (xQueueReceive(xVLMailbox, &vl_reading, portMAX_DELAY) == pdPASS)
        {
            printf("Timestamp: %u, VL53L0X: %u mm\n", 
                   vl_reading.xTimeStamp, vl_reading.distance);
        }
    }
}

void TaskCalculateAngle(void *pvParameters)
{
    tof10120_data_t tof_reading;
    vl53l0x_data_t vl_reading;

    while (true)
    {
        if (xQueueReceive(xTOFMailbox, &tof_reading, portMAX_DELAY) == pdPASS &&
            xQueueReceive(xVLMailbox, &vl_reading, portMAX_DELAY) == pdPASS)
        {
            float angle = calculate_seesaw_angle(tof_reading.distance, vl_reading.distance, 440);
            printf("Timestamp: %u, Seesaw Angle: %.2f degrees\n", 
                   tof_reading.xTimeStamp, angle);
        }
    }
}

float calculate_seesaw_angle(uint16_t d1, uint16_t d2, uint16_t length_mm)
{
    float delta_h = fabs((float)d1 - (float)d2);
    float theta_radians = atan(delta_h / length_mm);
    float theta_degrees = theta_radians * (180.0 / M_PI);
    return theta_degrees;
}
// End of file
