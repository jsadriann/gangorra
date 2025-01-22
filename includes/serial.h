#ifndef SERIAL_H
#define SERIAL_H

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "message.hpp"
#define UART_ID uart0
std::string uart_receive_data();
void vUartTask(void* pvParameters);
void vProcessTask(void* pvParameters);

#endif // SERIAL_H