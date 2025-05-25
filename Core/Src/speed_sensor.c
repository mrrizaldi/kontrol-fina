//#include "speed_sensor.h"
//#include "tim.h"  // TIM2 & TIM3 handle dari STM32CubeMX
//
//static uint32_t prev_count = 0;
//static uint32_t pulse_delta = 0;
//float rpm = 0;
//
//void SpeedSensor_Init(void) {
//    HAL_TIM_Base_Start(&htim2);  // Timer sebagai pulse counter
//}
//
//void SpeedSensor_Update(void) {
//    uint32_t curr_count = __HAL_TIM_GET_COUNTER(&htim2);
//
//    pulse_delta = (curr_count >= prev_count)
//                ? (curr_count - prev_count)
//                : (0xFFFF - prev_count + curr_count + 1);
//
//    prev_count = curr_count;
//
//    const uint8_t slot_count = 20;  // jumlah slot pada disk
//    float interval_sec = 0.1f;      // interval sampling = 100 ms
//    rpm = ((float)pulse_delta / slot_count) * (60.0f / interval_sec);
//}
//
//uint32_t SpeedSensor_GetPulseDelta(void) {
//    return pulse_delta;
//}
//
//float SpeedSensor_GetRPM(void) {
////    return rpm;
////	return 0;
//}
