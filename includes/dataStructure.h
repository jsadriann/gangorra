#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H
#include "mpu6050.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    accel accelData;
    gyro gyroData;
} MpuData_h;

typedef struct xMailboxManager{
    TickType_t xTimeStamp;
    MpuData_h mpu;
}MpuMailbox_t;

#endif // DATASTRUCTURE_H