#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include "mpu6050.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu6050.hpp"
#include "dataStructure.h"
void vTaskReadMpu(void *pvParameters);
void vTaskPrintMpu(void *pvParameters);

#endif //MPU6050_HANDLER_H