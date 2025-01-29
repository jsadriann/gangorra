#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motor.hpp"
#include "dataStructure.h"

void vTaskMotorControl(void *pvParameters);

#endif