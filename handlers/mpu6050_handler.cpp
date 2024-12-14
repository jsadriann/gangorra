#include "mpu6050_handler.h"
#include <stdio.h>

// Tarefa para capturar os dados do sensor
void readSensorTask(void *pvParameters) {
    QueueHandle_t sensorQueue = (QueueHandle_t) pvParameters;
    int16_t accel[3], gyro[3];

    while (1) {
        mpu6050_read(accel, gyro);

        // Envia os dados para a fila
        int16_t data[6] = {accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]};
        xQueueSend(sensorQueue, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(100)); // Leitura a cada 100ms
    }
}

// Tarefa para processar os dados do sensor
void processSensorDataTask(void *pvParameters) {
    QueueHandle_t sensorQueue = (QueueHandle_t) pvParameters;
    int16_t data[6];

    while (1) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {
            printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d\n",
                   data[0], data[1], data[2], data[3], data[4], data[5]);
        }
    }
}

// Função para criar as tarefas
void create_sensor_tasks(QueueHandle_t sensorQueue) {
    xTaskCreate(readSensorTask, "Sensor Reader", 256, (void *)sensorQueue, 1, NULL);
    xTaskCreate(processSensorDataTask, "Sensor Processor", 256, (void *)sensorQueue, 1, NULL);
}
