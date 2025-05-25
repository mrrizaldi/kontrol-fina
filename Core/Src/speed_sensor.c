#include "speed_sensor.h"
#include "tim.h"

static uint32_t prev_count = 0;
static uint32_t pulse_delta = 0;
static float current_rpm = 0.0f;

// Configuration constants
#define ENCODER_SLOTS 20        // Number of slots on the encoder disk
#define SAMPLE_PERIOD_SEC 0.1f  // 100ms sampling period

void SpeedSensor_Init(void) {
    // Timer is already initialized in tim.c
    // Just reset the counter
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    prev_count = 0;
    pulse_delta = 0;
    current_rpm = 0.0f;
}

void SpeedSensor_Update(void) {
    // Get current counter value
    uint32_t curr_count = __HAL_TIM_GET_COUNTER(&htim2);
    
    // Calculate pulse difference, handling overflow
    if (curr_count >= prev_count) {
        pulse_delta = curr_count - prev_count;
    } else {
        // Handle 16-bit counter overflow
        pulse_delta = (0x10000 - prev_count) + curr_count;
    }
    
    prev_count = curr_count;
    
    // Calculate RPM
    // Formula: RPM = (pulses_per_sample / slots_per_revolution) * (60 / sample_period)
    current_rpm = ((float)pulse_delta / ENCODER_SLOTS) * (60.0f / SAMPLE_PERIOD_SEC);
}

uint32_t SpeedSensor_GetPulseDelta(void) {
    return pulse_delta;
}

float SpeedSensor_GetRPM(void) {
    return current_rpm;
}