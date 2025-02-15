#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <motor.hpp>
#include "mpu6050.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu6050.hpp"
#define MESSAGE_MAX_LENGTH 256

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
    float angle;
}MpuMailbox_t;

typedef struct task_params {
    mpu6050* sensor;
    QueueHandle_t mailbox;
}mpu_params;

typedef struct motor_task_params {
    Motor* motor;
    QueueHandle_t mailbox;
}motor_params;

typedef struct processMessageEsp_params {
    QueueHandle_t receive;
    QueueHandle_t send;
}PMEParams_t;

typedef struct pid_task_paramas {
    QueueHandle_t mpuMailbox;
    QueueHandle_t leftMotorQueue;
    QueueHandle_t rightMotorQueue;
}PIDParams_t;

#endif // DATASTRUCTURE_H
