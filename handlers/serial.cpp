#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"  // Para utilizar o mutex
#include <cstdio>   // Para printf
#include <cstring>
#include "dataStructure.h"
extern float kp, ki, kd;
float akp, aki, akd;
extern float tof_r, tof_l, accel_x, accel_y, accel_z, angle, current;
extern float current_tof_r, current_tof_l, current_accel_x, current_accel_y, current_accel_z, current_angle, current_current;
#define UART_ID uart1
#define UART_TX_PIN 4  // GPIO 4 → TX
#define UART_RX_PIN 5  // GPIO 5 → RX

#define BAUD_RATE 115200
#define MESSAGE_MAX_LENGTH 256

// Definir o tipo da fila para usar char arrays (buffer fixo)
QueueHandle_t xMailboxMessage;
char received_message[MESSAGE_MAX_LENGTH];  // Buffer fixo para armazenar mensagens recebidas
int message_index = 0;  // Índice para armazenar no buffer

// Mutex para proteger o acesso à UART0
SemaphoreHandle_t xUartMutex;

// Função para configurar a UART0 na Raspberry Pi Pico
void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);  // Inicializa UART0
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);  // Configura o TX
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);  // Configura o RX
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);   // Formato de dados
    uart_set_hw_flow(UART_ID, false, false);            // Habilita controle de fluxo de hardware (RTS/CTS)
    uart_set_fifo_enabled(UART_ID, true);                // Habilita o FIFO da UART

    // Criar fila para mensagens
    xMailboxMessage = xQueueCreate(5, sizeof(received_message));
    if (xMailboxMessage == NULL) {
        printf("Erro ao criar mailbox!\n");
    }

    // Criar o mutex
    xUartMutex = xSemaphoreCreateMutex();
    if (xUartMutex == NULL) {
        printf("Erro ao criar o mutex da UART!\n");
    }
}

// Task responsavel por receber dados da ESP32 e adicionar na fila que recebimentos
void vTaskReceive(void *pvParameters) {
    QueueHandle_t xReceiveEsp = (QueueHandle_t)pvParameters;
    while (true) {
        if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) {
            if (uart_is_readable(UART_ID)) {
                char c = uart_getc(UART_ID);
                if (message_index < MESSAGE_MAX_LENGTH - 1) {
                    received_message[message_index++] = c;
                }
                //printf("%c\n", c);

                if (c == '/') {
                    received_message[message_index] = '\0';
                    // Verifica se a fila está recebendo a mensagem corretamente
                    if (xQueueSend(xReceiveEsp, &received_message, portMAX_DELAY) == pdPASS) {
                        printf("Mensagem adicionada à fila xReceiveEsp: %s\n", received_message);
                    } else {
                        printf("Falha ao enviar mensagem para a fila!\n");
                    }
                    message_index = 0;
                }
            }// }else{
            //     printf("Nenhum dado recebido\n");
            // }
            xSemaphoreGive(xUartMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Tarefa para exibir mensagens recebidas da ESP32
void vTaskDisplayMessage(void *pvParameters) {
    QueueHandle_t xReceiveEsp = (QueueHandle_t)pvParameters;
    char mes[MESSAGE_MAX_LENGTH];
    while (true) {
        // Se houver mensagens na fila, exiba-as
        if (xQueueReceive(xReceiveEsp, &mes, portMAX_DELAY) == pdPASS) {
            printf("Mensagem recebida da ESP32: %s\n", mes);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Aguarda 1 segundo
    }
}

void vTaskProcessMessageEsp(void *pvParameters) {
    PMEParams_t* pmeParams = (PMEParams_t*)pvParameters;
    QueueHandle_t xReceiveEsp = pmeParams->receive;
    QueueHandle_t xSendEsp = pmeParams->send;
    char receivedMessage[MESSAGE_MAX_LENGTH];

    while (true) {
        if (xQueueReceive(xReceiveEsp, &receivedMessage, portMAX_DELAY) == pdPASS) {
            // Verifica se a mensagem está bem formada
            if (receivedMessage[0] != '#' || receivedMessage[strlen(receivedMessage) - 1] != '/') {
                printf("Mensagem inválida!\n");
                continue;
            }

            // Remover o caractere '#' e '/' do início e fim
            char *start = receivedMessage + 1;  // Pula o '#'
            char *end = strchr(start, '/');  
            if (end) *end = '\0';  // Substitui '/' por fim de string

            // Separar o comando
            char *command = strtok(start, "$");
            if (!command) {
                printf("Erro ao extrair comando\n");
                continue;
            }

            // Separar o tipo
            char *type = strtok(NULL, "@");
            if (!type) {
                printf("Erro ao extrair type\n");
                continue;
            }

            // Separar os parâmetros
            char *parameters = strtok(NULL, ":");
            if (!parameters) {
                printf("Erro ao extrair parâmetros\n");
                continue;
            }

            // Extrair valores numéricos dos parâmetros
            if (sscanf(parameters, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",&akp, &aki, &akd, &tof_r, &tof_l, &accel_x, &accel_y, &accel_z, &angle, &current) != 7) {
                printf("Erro ao converter parâmetros\n");
                continue;
            }

            // Exibir os valores extraídos
            printf("Comando: %s\n", command);
            printf("Tipo: %s\n", type);
            printf("akp: %.2f, aki: %.2f, akd: %.2f,tof_r: %.2f, tof_l: %.2f, accel_x: %.2f, accel_y: %.2f, accel_z: %.2f, angle: %.2f, current: %.2f\n",
                   akp,aki,akd,tof_r, tof_l, accel_x, accel_y, accel_z, angle, current);
            if (strcmp(type, "0") == 0){
                char rMessage[MESSAGE_MAX_LENGTH];
                switch ((int)command)
                {
                case '0':
                    // Criar o pacote diretamente na variável received_message
                    snprintf(rMessage, MESSAGE_MAX_LENGTH, "#send$Response@ :%f,%f,%f,%f,%f,%f,%f,%f,%f,%f:/", 
                            kp, ki, kd, tof_r, tof_l, accel_x, accel_y, accel_z, angle, current);
                    break;
                default:
                    break;
                }
                if (!(strlen(rMessage) == 0)) {
                    // Envia a resposta
                    if (xQueueSend(xSendEsp, &rMessage, portMAX_DELAY) == pdPASS) {
                        printf("Mensagem enviada para a ESP32: %s\n", rMessage);
                    }
                    
                }
            }
        }
    }
}

// Tarefa para enviar mensagem pela a ESP32
void vTaskSendMessage(void *pvParameters) {
    QueueHandle_t xSendEsp = (QueueHandle_t)pvParameters;
    char mes[MESSAGE_MAX_LENGTH];
    while (true) {
        if (xQueueReceive(xSendEsp, &mes, portMAX_DELAY) == pdPASS) {
            for (int i = 0; mes[i] != '\0'; i++) {
                if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) {
                    uart_putc(UART_ID, mes[i]);  // Envia um caractere por vez
                    xSemaphoreGive(xUartMutex);    // Libera o UART após cada caractere
                }
                vTaskDelay(pdMS_TO_TICKS(50));  // Pequeno delay para evitar congestionamento
            }
            
            printf("Mensagem enviada para a ESP32 depois de quebrar o pacote: %s\n", mes);  // Aguarda 5 segundos antes de enviar novamente
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vTaskGenerateMessage(void *pvParameters) {
    QueueHandle_t xSendEsp = (QueueHandle_t)pvParameters;
    char message[MESSAGE_MAX_LENGTH];

    while (true) {
        // Exemplo de mensagem no formato especificado
        snprintf(message, MESSAGE_MAX_LENGTH, "#0$0@:1.0,1.0,1.0,1.4,2.5,3.0,3.1,30,24,20:/");
        
        // Enviar para a fila
        if (xQueueSend(xSendEsp, &message, portMAX_DELAY) == pdPASS) {
            printf("Mensagem gerada e enviada para a fila xSendEsp: %s\n", message);
        } else {
            printf("Falha ao enviar mensagem para a fila!\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));  // Aguarda 5 segundos antes de enviar novamente
    }
}

