#ifndef MYTASKS_H
#define MYTASKS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"


void vTaskMain(void *pvParameters);
void vTaskBLE(void *pvParameters);
void vTaskADC(void *pvParameters);
void vTaskCmd(void *pvParameters);
void vTaskCmdMot(void *pvParameters);
void vTaskCmdSt(void *pvParameters);
void vTaskCmdServo(void *pvParameters);
void vTaskCmdLeds(void *pvParameters);
void vTaskCmdDist(void *pvParameters);
void vTaskLedsBlink(void *pvParameters);

uint8_t sendPackToBLE(const char * blepack);



#endif
