#ifndef SERIAL_H
#define SERIAL_H

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string>

// Definições da UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Mailbox para armazenar mensagens recebidas
extern QueueHandle_t xMailboxMessage;
extern SemaphoreHandle_t xMutex;

// Configuração da UART e interrupções
void setup_uart();

// Tarefa para exibir mensagens recebidas
void vTaskDisplayMessage(void *pvParameters);

void vTaskSendMessage(void *pvParameters);

void vTaskReceive(void *pvParameters);
void vTaskGenerateMessage(void *pvParameters);
void vTaskProcessMessageEsp(void *pvParameters);

#endif // SERIAL_H
