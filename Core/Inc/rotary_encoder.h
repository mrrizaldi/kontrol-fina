#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "stm32f1xx_hal.h"  // Sesuaikan dengan seri STM32 yang digunakan

typedef struct {
    GPIO_TypeDef *CLK_GPIO;
    GPIO_TypeDef *DT_GPIO;
    GPIO_TypeDef *SW_GPIO;
    uint16_t CLK_PIN;
    uint16_t DT_PIN;
    uint16_t SW_PIN;
    int16_t position;
    uint8_t buttonPressed;
} RotaryEncoder;

// Deklarasi fungsi
void RotaryEncoder_Init(GPIO_TypeDef *CLK_GPIO, uint16_t CLK_PIN,
                        GPIO_TypeDef *DT_GPIO, uint16_t DT_PIN,
                        GPIO_TypeDef *SW_GPIO, uint16_t SW_PIN);
void RotaryEncoder_Update(void);
void RotaryEncoder_Reset(void);
uint8_t RotaryEncoder_IsPressed(void);
int16_t RotaryEncoder_GetPosition(void);

#endif
