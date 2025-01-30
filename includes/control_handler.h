#ifndef CONTROL_HANDLER_H
#define CONTROL_HANDLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
#include "hardware/adc.h"

#define MIN_DUTY 30   // Duty cycle mínimo em porcentagem
#define MAX_DUTY 60    // Duty cycle máximo em porcentagem

void potentiometerTask(void *pvParameters);

#endif //CONTROL_HANDLER_H