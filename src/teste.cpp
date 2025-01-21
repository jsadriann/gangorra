#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
#include "mpu6050_handler.h"
#include "serial.h"



// Definição do heap para o FreeRTOS
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

#define QUEUE_SIZE 1 // Define o tamanho da fila (pode ser ajustado conforme necessário)

QueueHandle_t xMailbox; // Fila para armazenar dados do sensor

// Prototipação das tarefas
//void TaskUpdate(void *pvParameters);
//void TaskRead(void *pvParameters);

int main() {

    stdio_init_all();
    //inicializando a uart
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    //cria o Mailbox pra cada sensor
    xMailbox = xQueueCreate(1, sizeof(MpuMailbox_t));
    //inicializa o i2c, aqui não to utilizando a superclass do i2c ainda não
    i2c_inst_t *i2c_port = i2c0;
    //cria uma instancia do mpu
    mpu6050 sensor(i2c_port, 4, 5);
    //cria os parametros das task que envolvam o mpu
    //sendo que os parametros atuais são uma instancia da class mpu6050
    //e o outro é o mailbox do mpu
    mpu_params mpu;
    mpu.sensor = &sensor;
    mpu.mailbox = xMailbox;

    //cria as tarefas de ler o sensor e adicionar o dado no mailbox
    // e a outra de ler do mailbox pra testar se ta ok a comunicação entre as tasks
    xTaskCreate(vTaskReadMpu, "Task1", 256, &mpu, 1, NULL); // Tarefa 1 com prioridade 1
    xTaskCreate(vTaskPrintMpu, "Task2", 256, &mpu, 1, NULL); // Tarefa 2 com prioridade 1
    xTaskCreate(vTaskReceiveSerialData, "Task3", 256, NULL, 1, NULL);
    xTaskCreate(vTaskSendSerialData, "Task4", 256, NULL, 1, NULL);

    //escalona as tarefas
    vTaskStartScheduler();

    while (1) {
        printf("Falha ao iniciar o scheduler do FreeRTOS.\n");
        sleep_ms(1000);
    }
    return 0;
}
// Implementação da Tarefa 1
// void TaskUpdate(void *pvParameters) {
//     mpu_params* mpu = (mpu_params*)pvParameters;
//     QueueHandle_t mailbox = mpu->mailbox;
//     mpu6050* sensor = mpu->sensor;

//     MpuMailbox_t mailboxData;
//     sensor->getAccel(&mailboxData.mpu.accelData);
//     sensor->getGyro(&mailboxData.mpu.gyroData);
//     mailboxData.xTimeStamp = xTaskGetTickCount();
//     while (1) {
//         // Envia uma mensagem para a mailbox
//         if (xQueueOverwrite(mailbox, &mailboxData) == pdPASS) {
//             printf("Producer: MPU Send\n");
//         } else {
//             printf("Producer: Falha ao enviar para a mailbox\n");
//         }
//         // Aguarda 1 segundo antes de enviar a próxima mensagem
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         sensor->getAccel(&mailboxData.mpu.accelData);
//         sensor->getGyro(&mailboxData.mpu.gyroData);
//         mailboxData.xTimeStamp = xTaskGetTickCount();
//     }
// }

// // Implementação da Tarefa 2
// void TaskRead(void *pvParameters) {
//     mpu_params* mpu = (mpu_params*)pvParameters;
//     QueueHandle_t mailbox = mpu->mailbox;
//     mpu6050* sensor = mpu->sensor;

//     MpuMailbox_t receivedMessage;
//     while (1) {
//         // Lê o valor da mailbox (aguarda indefinidamente até receber)
//         if (xQueueReceive(mailbox, &receivedMessage, portMAX_DELAY) == pdPASS) {
//             printf("Dados do MPU6050\n");
//             printf("gyro_X = %d\n", receivedMessage.mpu.gyroData.gyro_x);
//             printf("gyro_Y = %d\n", receivedMessage.mpu.gyroData.gyro_y);
//             printf("gyro_Z = %d\n", receivedMessage.mpu.gyroData.gyro_z);
//             printf("accel_X = %d\n", receivedMessage.mpu.accelData.accel_x);
//             printf("accel_Y = %d\n", receivedMessage.mpu.accelData.accel_y);
//             printf("accel_Z = %d\n", receivedMessage.mpu.accelData.accel_z);
//             printf("Timestamp: %d\n", receivedMessage.xTimeStamp); // Imprime o timestamp do recebimento
//             printf("Memória Heap livre: %u bytes\n", (unsigned int)xPortGetFreeHeapSize());
//         } else {
//             printf("Consumer: Falha ao receber da mailbox\n");
//         }

//         // Aguarda 500 ms antes de tentar novamente
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }


#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
//#include "i2c_sensors.hpp"



// Definição do heap para o FreeRTOS
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

#define QUEUE_SIZE 1 // Define o tamanho da fila (pode ser ajustado conforme necessário)

QueueHandle_t xMailboxMpu; // Fila para armazenar dados do sensor

// Prototipação das tarefas
void TaskUpdate(void *pvParameters);
void TaskRead(void *pvParameters);

int main() {

    stdio_init_all();
    xMailboxMpu = xQueueCreate(1, sizeof(MpuMailbox_t));

    i2c_inst_t* i2c_port = i2c0;  // Ou i2c1, dependendo do barramento
    uint sda_pin = 0;  // Pino SDA
    uint scl_pin = 1;  // Pino SCL
    uint32_t baudrate = 400000;  // Baud rate

    //i2c_sensors sensor(i2c_port, sda_pin, scl_pin, baudrate);
    //sensor.initialize();

    mpu6050 mpu(i2c_port, sda_pin, scl_pin);
    /*
    mpu_params params_mpu;
    params_mpu.sensor = &mpu;
    params_mpu.mailbox = xMailboxMpu;
    xTaskCreate(TaskUpdate, "Task1", 256, &params_mpu, 1, NULL); // Tarefa 1 com prioridade 1
    xTaskCreate(TaskRead, "Task2", 256, &params_mpu, 1, NULL); // Tarefa 2 com prioridade 1

    // // Iniciar o agendador do FreeRTOS
    //vTaskStartScheduler();
    */
    // // Caso o scheduler falhe
    while (1) {
        printf("Falha ao iniciar o scheduler do FreeRTOS.\n");
        sleep_ms(1000);
    }
    return 0;

}
// Implementação da Tarefa 1
void TaskUpdate(void *pvParameters) {
    task_params* tparams = (task_params*)pvParameters;
    mpu6050* sensor = tparams->sensor;
    QueueHandle_t mailbox = tparams->mailbox;
    MpuMailbox_t mailboxData;
    sensor->getAccel(&mailboxData.mpu.accelData);
    sensor->getGyro(&mailboxData.mpu.gyroData);
    mailboxData.xTimeStamp = xTaskGetTickCount();
    while (1) {
        // Envia uma mensagem para a mailbox
        if (xQueueOverwrite(mailbox, &mailboxData) == pdPASS) {
            printf("Producer: MPU Send\n");
        } else {
            printf("Producer: Falha ao enviar para a mailbox\n");
        }
        // Aguarda 1 segundo antes de enviar a próxima mensagem
        vTaskDelay(pdMS_TO_TICKS(1000));
        sensor->getAccel(&mailboxData.mpu.accelData);
        sensor->getGyro(&mailboxData.mpu.gyroData);
        mailboxData.xTimeStamp = xTaskGetTickCount();
    }
}

// Implementação da Tarefa 2
void TaskRead(void *pvParameters) {
    task_params* tparams = (task_params*)pvParameters;
    mpu6050* sensor = tparams->sensor;
    QueueHandle_t mailbox = tparams->mailbox;
    
    MpuMailbox_t receivedMessage;
    while (1) {
        // Lê o valor da mailbox (aguarda indefinidamente até receber)
        if (xQueueReceive(mailbox, &receivedMessage, portMAX_DELAY) == pdPASS) {
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
