#ifndef ROBOTS_DETECTION_H
#define ROBOTS_DETECTION_H

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "Robots_Control.h"
#include "User_typedef.h"
#include "DJI_C_IMU.h"
#include "RM_JudgeSystem.h"
#include "Sensor.h"

void IMUFPS_0_Emergency_status(void);
void Offline_Clear_FPS(void);


#endif
