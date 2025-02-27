#ifndef TOF_HANDLER_HPP
#define TOF_HANDLER_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tof.hpp"
#include "VL53L0X.hpp"

typedef struct {
    uint16_t distance;
    TickType_t xTimeStamp;
} tof10120_data_t;

typedef struct {
    uint16_t distance;
    TickType_t xTimeStamp;
} vl53l0x_data_t;

extern QueueHandle_t xTOFMailbox;
extern QueueHandle_t xVLMailbox;

void TaskVLUpdate(void *pvParameters);
void TaskUpdate(void *pvParameters);
void TaskRead(void *pvParameters);

#endif // TOF_HANDLER_HPP
