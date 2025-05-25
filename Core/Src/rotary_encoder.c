#include "rotary_encoder.h"

static RotaryEncoder encoder;

void RotaryEncoder_Init(GPIO_TypeDef *CLK_GPIO, uint16_t CLK_PIN,
                        GPIO_TypeDef *DT_GPIO, uint16_t DT_PIN,
                        GPIO_TypeDef *SW_GPIO, uint16_t SW_PIN) {
    encoder.CLK_GPIO = CLK_GPIO;
    encoder.DT_GPIO = DT_GPIO;
    encoder.SW_GPIO = SW_GPIO;
    encoder.CLK_PIN = CLK_PIN;
    encoder.DT_PIN = DT_PIN;
    encoder.SW_PIN = SW_PIN;
    encoder.position = 0;
    encoder.buttonPressed = 0;
}

void RotaryEncoder_Update(void) {
    static uint8_t lastState = 0;
    static uint32_t lastTime = 0;
    uint32_t currentTime = HAL_GetTick();  // Ambil waktu sekarang dalam ms

    if (currentTime - lastTime < 1) return;  // Debounce 5 ms
    lastTime = currentTime;

    uint8_t clkState = HAL_GPIO_ReadPin(encoder.CLK_GPIO, encoder.CLK_PIN);
    uint8_t dtState = HAL_GPIO_ReadPin(encoder.DT_GPIO, encoder.DT_PIN);

    if (clkState == GPIO_PIN_SET && lastState == GPIO_PIN_RESET) {
        if (dtState == GPIO_PIN_RESET) {
            encoder.position++;
        } else {
            encoder.position--;
        }
    }
    lastState = clkState;
}

void RotaryEncoder_Reset(void) {
    encoder.position = 0;
}

uint8_t RotaryEncoder_IsPressed(void) {
    if (HAL_GPIO_ReadPin(encoder.SW_GPIO, encoder.SW_PIN) == GPIO_PIN_RESET) {
        encoder.buttonPressed = 1;
    } else {
        encoder.buttonPressed = 0;
    }
    return encoder.buttonPressed;
}

int16_t RotaryEncoder_GetPosition(void) {
    return encoder.position;
}
