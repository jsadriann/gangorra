#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"



// Definição do heap para o FreeRTOS
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

#define QUEUE_SIZE 1 // Define o tamanho da fila (pode ser ajustado conforme necessário)

QueueHandle_t xMailbox; // Fila para armazenar dados do sensor

// Prototipação das tarefas
void TaskUpdate(void *pvParameters);
void TaskRead(void *pvParameters);

int main() {

    stdio_init_all();
    // // sensorDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
    // // if (sensorDataQueue == NULL) {
    // //     printf("Falha ao criar a fila. Heap livre: %d bytes\n", xPortGetFreeHeapSize());
    // //     while (1); // Trava o sistema
    // // }
    // xMailbox = xQueueCreate(1, sizeof(MpuMailbox_t));

    // // Criação das tarefas
    // xTaskCreate(TaskUpdate, "Task1", 256, NULL, 1, NULL); // Tarefa 1 com prioridade 1
    // xTaskCreate(TaskRead, "Task2", 256, NULL, 1, NULL); // Tarefa 2 com prioridade 1

    // // Iniciar o agendador do FreeRTOS
    // vTaskStartScheduler();

    // // Caso o scheduler falhe
    // while (1) {
    //     printf("Falha ao iniciar o scheduler do FreeRTOS.\n");
    //     sleep_ms(1000);
    // }
    // return 0;

    // Defina os pinos de SDA e SCL
    const uint sda_pin = 4; // Número do pino para SDA
    const uint scl_pin = 5; // Número do pino para SCL

    // Porta I2C a ser usada
    i2c_inst_t *i2c_port = i2c0;

    // Inicialize a porta I2C
    i2c_init(i2c_port, 400 * 1000); // Frequência de 400 kHz
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(scl_pin);

    // Crie uma instância do sensor MPU6050
    mpu6050 sensor(i2c_port, sda_pin, scl_pin);

    // Variáveis para armazenar dados do acelerômetro e giroscópio
    accel accelData = {0};
    gyro gyroData = {0};

    // Use a instância do sensor para obter e imprimir os dados
    while (true) {
        sensor.getAccel(&accelData); // Leia os dados do acelerômetro
        sensor.getGyro(&gyroData);   // Leia os dados do giroscópio

        // Imprima os dados formatados
        sensor.print_raw_data(accelData, gyroData);

        sleep_ms(1000);
    }

    return 0;

}
// Implementação da Tarefa 1
void TaskUpdate(void *pvParameters) {
    (void)pvParameters;
    MpuMailbox_t mailboxData;
    mailboxData.mpu.accelData.accel_x= 1;
    mailboxData.mpu.accelData.accel_y= 2;
    mailboxData.mpu.accelData.accel_z= 3;
    mailboxData.mpu.gyroData.gyro_x = 4;
    mailboxData.mpu.gyroData.gyro_y = 5;
    mailboxData.mpu.gyroData.gyro_z = 6;
    mailboxData.xTimeStamp = xTaskGetTickCount();
    while (1) {
        // Envia uma mensagem para a mailbox
        if (xQueueOverwrite(xMailbox, &mailboxData) == pdPASS) {
            printf("Producer: MPU Send\n");
        } else {
            printf("Producer: Falha ao enviar para a mailbox\n");
        }
        // Aguarda 1 segundo antes de enviar a próxima mensagem
        vTaskDelay(pdMS_TO_TICKS(1000));
        mailboxData.mpu.accelData.accel_x= 1;
        mailboxData.mpu.accelData.accel_y= 2;
        mailboxData.mpu.accelData.accel_z= 3;
        mailboxData.mpu.gyroData.gyro_x = 4;
        mailboxData.mpu.gyroData.gyro_y = 5;
        mailboxData.mpu.gyroData.gyro_z = 6;
        mailboxData.xTimeStamp = xTaskGetTickCount();
    }
}

// Implementação da Tarefa 2
void TaskRead(void *pvParameters) {
    (void)pvParameters; // Parâmetro não utilizado
    MpuMailbox_t receivedMessage;
    while (1) {
        // Lê o valor da mailbox (aguarda indefinidamente até receber)
        if (xQueueReceive(xMailbox, &receivedMessage, portMAX_DELAY) == pdPASS) {
            printf("Dados do MPU6050\n");
            printf("gyro_X = %d\n", receivedMessage.mpu.gyroData.gyro_x);
            printf("gyro_Y = %d\n", receivedMessage.mpu.gyroData.gyro_y);
            printf("gyro_Z = %d\n", receivedMessage.mpu.gyroData.gyro_z);
            printf("accel_X = %d\n", receivedMessage.mpu.accelData.accel_x);
            printf("accel_Y = %d\n", receivedMessage.mpu.accelData.accel_y);
            printf("accel_Z = %d\n", receivedMessage.mpu.accelData.accel_z);
            printf("Timestamp: %d\n", receivedMessage.xTimeStamp); // Imprime o timestamp do recebimento
        } else {
            printf("Consumer: Falha ao receber da mailbox\n");
        }

        // Aguarda 500 ms antes de tentar novamente
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
