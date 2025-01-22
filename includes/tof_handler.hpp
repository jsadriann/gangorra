#ifndef TOF_HANDLER_H
#define TOF_HANDLER_H

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tof.hpp"
#include "dataStructure.h"

void TaskUpdate(void *pvParameters);
void TaskRead(void *pvParameters);


#endif // TOF_HANDLER_H