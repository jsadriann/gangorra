#ifndef SERIAL_H
#define SERIAL_H

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#define UART_ID uart0          // Seleciona a UART0
#define BAUD_RATE 9600         // Taxa de transmissão (baud rate)
#define TX_PIN 0               // Pino TX (pode ser ajustado)
#define RX_PIN 1   
// Tarefa para enviar dados pela UART
void vTaskSendSerialData(void *pvParameters);

// Tarefa para receber dados pela UART
void vTaskReceiveSerialData(void *pvParameters);

#endif // SERIAL_H