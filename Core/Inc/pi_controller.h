#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "main.h"

typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float setpoint;     // Desired RPM
    float integral;     // Integral accumulator
    float prev_error;   // Previous error for derivative (if needed later)
    float output_min;   // Minimum output limit (0%)
    float output_max;   // Maximum output limit (100%)
    float dt;           // Time step in seconds
} PI_Controller;

// Function prototypes
void PI_Controller_Init(PI_Controller *pi, float kp, float ki, float setpoint);
float PI_Controller_Update(PI_Controller *pi, float current_value);
void PI_Controller_SetSetpoint(PI_Controller *pi, float new_setpoint);
void PI_Controller_Reset(PI_Controller *pi);

#endif // PI_CONTROLLER_H