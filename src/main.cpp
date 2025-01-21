// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "dataStructure.h"
// #include "mpu6050_handler.h"
// #include "serial.h"
// #include "mpu6050.hpp"
// __attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
// int main() {
//     stdio_init_all();
//     // Defina os pinos I2C e a porta I2C
//     i2c_inst *i2c_port = i2c0;  // ou i2c1, dependendo da sua configuração
//     uint16_t sda_pin = 0;       // pino SDA
//     uint16_t scl_pin = 1;       // pino SCL

//     // Cria uma instância do sensor MPU6050
//     mpu6050 sensor(i2c_port, sda_pin, scl_pin);

//     // Inicializa as estruturas para armazenar os dados do acelerômetro e giroscópio
//     accel accelData;
//     gyro gyroData;

//     // Loop para imprimir os dados do sensor
//     while (true) {
//         // Imprime os dados brutos (acelerômetro e giroscópio)
//         sensor.getAccel(&accelData);
//         sensor.getGyro(&gyroData);
//         sensor.print_raw_data(accelData, gyroData);

//         // Espera 1 segundo antes de ler novamente
//         sleep_ms(1000);
//     }

//     return 0;  // Nunca deve chegar aqui, pois o loop é infinito
// }


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
    mpu6050 sensor(i2c_port, 0, 1);
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