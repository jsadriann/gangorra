#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
#include "mpu6050_handler.h"
#include "serial.h"
#include "message.hpp"
#include "motor_handler.h"
#include "control_handler.h"
#include "kalman_filter_handler.h"

#define LOG_BUFFER_SIZE 512
#define RUN_TIME_BUFFER_SIZE 512

void vSystemLogTask(void *pvParameters);


// Definição do heap para o FreeRTOS
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

#define QUEUE_SIZE 1 // Define o tamanho da fila (pode ser ajustado conforme necessário)

QueueHandle_t xMailbox; // Fila para armazenar dados do Mpu
QueueHandle_t xMessageEsp;
QueueHandle_t xMotor;

int main() {
    stdio_init_all();

    //cria o Mailbox pra cada sensor
    xMailbox = xQueueCreate(1, sizeof(MpuMailbox_t));
    xMessageEsp = xQueueCreate(10, sizeof(Message));
    xMotor = xQueueCreate(10, sizeof(int));

    //inicializa o i2c, aqui não to utilizando a superclass do i2c ainda não
    i2c_inst_t *i2c_port = i2c0;
    //cria uma instancia do mpu
    mpu6050 sensor(i2c_port, 0, 1);
    //cria os parametros das task que envolvam o mpu
    //sendo que os parametros atuais são uma instancia da class mpu6050
    //e o outro é o mailbox do mpu
    mpu_params mpu;
    mpu.sensor = &sensor;
    mpu.mailbox = xMailbox;

    char buffer[0xffff];

    Motor left_motor(300, 15);

    motor_params left_motor_params;
    left_motor_params.motor = &left_motor;
    left_motor_params.mailbox = xMotor;

    //cria as tarefas de ler o sensor e adicionar o dado no mailbox
    // e a outra de ler do mailbox pra testar se ta ok a comunicação entre as tasks
    xTaskCreate(vTaskReadMpu, "Task MPU read", 256, &mpu, 2, NULL); // Tarefa 1 com prioridade 1
    xTaskCreate(vTaskPrintMpu, "Task MPU print", 256, &mpu, 2, NULL); // Tarefa 2 com prioridade 1
    xTaskCreate(vUartTask, "Task UART", 1000, xMessageEsp, 1, NULL);
    xTaskCreate(vProcessTask, "Task Process", 1000, xMessageEsp, 1, NULL);
    //xTaskCreate(vTaskReceiveSerialData, "Task UART receive", 256, NULL, 1, NULL);
    //xTaskCreate(vTaskSendSerialData, "Task UART send", 256, NULL, 1, NULL);
    xTaskCreate(vSystemLogTask,"Task Log", 256, buffer, 1, NULL);
    xTaskCreate(vTaskMotorControl, "Left Motor Task", 1000, &left_motor_params, 2, NULL);
    xTaskCreate(potentiometerTask, "Potentiometer Task", 1000, xMotor, 2, NULL);

    //escalona as tarefas
    vTaskStartScheduler();

    while (1) {
        printf("Falha ao iniciar o scheduler do FreeRTOS.\n");
        sleep_ms(1000);
    }
    return 0;
}

void vSystemLogTask(void *pvParameters) {
    char* pcWrite = (char*)pvParameters;
    const TickType_t logInterval = pdMS_TO_TICKS(4000); // Log a cada 2 segundos

    while (true) {
        printf("\n================= ESTADO DO SISTEMA =================\n");

        // Log do uso de memória heap
        size_t freeHeap = xPortGetFreeHeapSize();
        size_t minHeap = xPortGetMinimumEverFreeHeapSize();
        printf("Uso de Memória:\n");
        printf("Heap Livre: %lu bytes\n", freeHeap);
        printf("Heap Mínimo Já Registrado: %lu bytes\n\n", minHeap);

        // Log do estado das tasks
        printf("Estado das Tasks:\n");
        printf("Task Name\tStatus\tPri\tStack\tTask\tCoreAf#\n");
        vTaskList(pcWrite);
        printf("%s\n", pcWrite);

        // Estatísticas de tempo de execução (se configurado)
#if configGENERATE_RUN_TIME_STATS
        printf("Tempo de Execução das Tasks:\n");
        vTaskGetRunTimeStats(pcWrite);
        printf("%s\n", pcWrite);
#endif

        printf("=====================================================\n");

        // Delay para o próximo log
        vTaskDelay(logInterval);
    }
}