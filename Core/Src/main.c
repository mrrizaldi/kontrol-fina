/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Conveyor PI Controller
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "i2c-lcd.h"
#include "rotary_encoder.h"
#include "speed_sensor.h"
#include "pi_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_RPM 100.0f
#define CONTROL_PERIOD_MS 100
#define PWM_MAX_VALUE 24  // TIM1 period is 25-1, so max CCR value is 24

// PI Controller tuning parameters - adjust these for optimal performance
#define KP_GAIN 0.8f      // Proportional gain
#define KI_GAIN 0.3f      // Integral gain
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PI_Controller motor_pi;
float current_rpm = 0.0f;
float duty_cycle = 0.0f;
uint32_t last_control_time = 0;
uint32_t last_display_time = 0;
char lcd_buffer[20];

// System status
typedef enum {
    SYSTEM_INIT,
    SYSTEM_RUNNING,
    SYSTEM_STOPPED
} SystemState;

SystemState system_state = SYSTEM_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateDisplay(void);
void HandleRotaryEncoder(void);
void MotorControl_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD
  lcd_init();
  HAL_Delay(100);
  
  // Display startup message
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Conveyor PI Ctrl");
  lcd_put_cur(1, 0);
  lcd_send_string("Initializing...");
  HAL_Delay(2000);

  // Initialize rotary encoder
  RotaryEncoder_Init(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_3);

  // Initialize speed sensor
  SpeedSensor_Init();

  // Initialize PI controller
  PI_Controller_Init(&motor_pi, KP_GAIN, KI_GAIN, TARGET_RPM);

  // Start PWM and timer
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  // Initialize timing
  last_control_time = HAL_GetTick();
  last_display_time = HAL_GetTick();
  
  // Set initial PWM to 0
  TIM1->CCR1 = 0;
  
  system_state = SYSTEM_RUNNING;

  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    // Update rotary encoder
    RotaryEncoder_Update();
    HandleRotaryEncoder();
    
    // Run control loop every 100ms
    if ((current_time - last_control_time) >= CONTROL_PERIOD_MS) {
      MotorControl_Update();
      last_control_time = current_time;
    }
    
    // Update display every 500ms
    if ((current_time - last_display_time) >= 500) {
      UpdateDisplay();
      last_display_time = current_time;
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

void MotorControl_Update(void) {
  // Update speed sensor
  SpeedSensor_Update();
  current_rpm = SpeedSensor_GetRPM();
  
  // Run PI controller
  duty_cycle = PI_Controller_Update(&motor_pi, current_rpm);
  
  // Convert duty cycle percentage to timer compare value
  uint32_t ccr_value = (uint32_t)((duty_cycle / 100.0f) * PWM_MAX_VALUE);
  
  // Limit CCR value
  if (ccr_value > PWM_MAX_VALUE) {
    ccr_value = PWM_MAX_VALUE;
  }
  
  // Apply to motor
  TIM1->CCR1 = ccr_value;
}

void UpdateDisplay(void) {
  // Line 1: Current RPM and Setpoint
  lcd_put_cur(0, 0);
  snprintf(lcd_buffer, sizeof(lcd_buffer), "RPM:%3.0f SP:%3.0f", current_rpm, motor_pi.setpoint);
  lcd_send_string(lcd_buffer);
  
  // Line 2: Duty cycle and system status
  lcd_put_cur(1, 0);
  if (system_state == SYSTEM_RUNNING) {
    snprintf(lcd_buffer, sizeof(lcd_buffer), "PWM:%2.0f%% RUNNING", duty_cycle);
  } else {
    snprintf(lcd_buffer, sizeof(lcd_buffer), "PWM:%2.0f%% STOPPED", duty_cycle);
  }
  lcd_send_string(lcd_buffer);
}

void HandleRotaryEncoder(void) {
  static int16_t last_position = 0;
  static uint32_t last_button_time = 0;
  
  int16_t current_position = RotaryEncoder_GetPosition();
  uint32_t current_time = HAL_GetTick();
  
  // Handle rotation for setpoint adjustment
  int16_t position_change = current_position - last_position;
  if (position_change != 0) {
    float new_setpoint = motor_pi.setpoint + (position_change * 5.0f); // 5 RPM per click
    
    // Limit setpoint range
    if (new_setpoint < 0.0f) new_setpoint = 0.0f;
    if (new_setpoint > 300.0f) new_setpoint = 300.0f;
    
    PI_Controller_SetSetpoint(&motor_pi, new_setpoint);
    last_position = current_position;
  }
  
  // Handle button press for start/stop
  if (RotaryEncoder_IsPressed() && (current_time - last_button_time) > 500) {
    if (system_state == SYSTEM_RUNNING) {
      system_state = SYSTEM_STOPPED;
      TIM1->CCR1 = 0;  // Stop motor
      PI_Controller_Reset(&motor_pi);  // Reset controller
    } else {
      system_state = SYSTEM_RUNNING;
    }
    last_button_time = current_time;
  }
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */