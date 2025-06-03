#pragma once
#include "cmsis_os.h"

extern osThreadId_t insTaskHandle;
extern osThreadId_t motorTaskHandle;

void OSTaskInit(void);
void StartINSTASK(void *argument);
void StartTRANSTASK(void *argument);
void StartMOTORTASK(void *argument);