#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include "main.h"

void SpeedSensor_Init(void);
void SpeedSensor_Update(void);
uint32_t SpeedSensor_GetPulseDelta(void);
float SpeedSensor_GetRPM(void);

#endif // SPEED_SENSOR_H