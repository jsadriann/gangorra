#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu6050.h"


void create_sensor_tasks(QueueHandle_t sensorQueue);

#endif // SENSOR_HANDLER_H