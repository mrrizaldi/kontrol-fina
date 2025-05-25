#include "pi_controller.h"

void PI_Controller_Init(PI_Controller *pi, float kp, float ki, float setpoint) {
    pi->kp = kp;
    pi->ki = ki;
    pi->setpoint = setpoint;
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->output_min = 0.0f;     // 0% duty cycle
    pi->output_max = 100.0f;   // 100% duty cycle
    pi->dt = 0.1f;             // 100ms update rate
}

float PI_Controller_Update(PI_Controller *pi, float current_value) {
    // Calculate error
    float error = pi->setpoint - current_value;
    
    // Update integral term with anti-windup
    float potential_integral = pi->integral + error * pi->dt;
    
    // Calculate proportional term
    float proportional = pi->kp * error;
    
    // Calculate integral term
    float integral_term = pi->ki * potential_integral;
    
    // Calculate total output
    float output = proportional + integral_term;
    
    // Apply output limits and anti-windup
    if (output > pi->output_max) {
        output = pi->output_max;
        // Don't update integral if we're saturating high
    } else if (output < pi->output_min) {
        output = pi->output_min;
        // Don't update integral if we're saturating low
    } else {
        // Only update integral if we're not saturating
        pi->integral = potential_integral;
    }
    
    return output;
}

void PI_Controller_SetSetpoint(PI_Controller *pi, float new_setpoint) {
    pi->setpoint = new_setpoint;
}

void PI_Controller_Reset(PI_Controller *pi) {
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
}