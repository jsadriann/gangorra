#ifndef TOF_HANDLER_HPP
#define TOF_HANDLER_HPP

#include <cstdio>
#include <cmath>
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
void TaskCalculateAngle(void *pvParameters);
float calculate_seesaw_angle(uint16_t d1, uint16_t d2, uint16_t length_mm);

#endif // TOF_HANDLER_HPP
