#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H
#include "mpu6050.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu6050.hpp"

typedef struct {
    accel accelData;
    gyro gyroData;
} MpuData_h;

typedef struct {
    uint16_t distance; // Distance storage
    TickType_t xTimeStamp;
} tof_data_t;

typedef struct xMailboxManager{
    TickType_t xTimeStamp;
    MpuData_h mpu;
}MpuMailbox_t;

typedef struct task_params {
    mpu6050* sensor;
    QueueHandle_t mailbox;
}mpu_params;

#endif // DATASTRUCTURE_H
