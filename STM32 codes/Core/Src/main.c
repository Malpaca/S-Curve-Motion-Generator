/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>

#include "sequence.h"
#include "servo_map.h"
#include "buttons.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIP_CHANNEL    TIM_CHANNEL_1
#define KNEE_CHANNEL   TIM_CHANNEL_2
#define ANKLE_CHANNEL  TIM_CHANNEL_3

//HAL Tick are 1ms each
#define CONTROL_PERIOD 10u
#define OLED_PERIOD 250u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t raw_adc[1];

static sequence_t leg_seq;
static servo_cal_t hip_cal;
static servo_cal_t knee_cal;
static servo_cal_t ankle_cal;
static scurve_mode_t selected_mode = SCURVE_QUINTIC_SMOOTHERSTEP;
static float speed_scale = 1;
static uint32_t last_tick_ms;
static uint32_t screen_last_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_mode_text(void);
void update_speed_text(void);
void update_active_text(bool active);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //Start Timer Channels
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  //Center Servos
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
  //Button init
  buttons_init();
  ssd1306_Init();
  update_mode_text();
  update_speed_text();
  //Calibrate Servo angle range and pulse
  servo_cal_init(&hip_cal,  0.0f, 180.0f, 500, 2500, false);
  servo_cal_init(&knee_cal,  0.0f, 180.0f, 500, 2500, true);
  servo_cal_init(&ankle_cal,  0.0f, 180.0f, 500, 2500, true);

  const sequence_point_t triangle[] = {
          /* hip, knee, unused */
          { .value = { 40.914f,  82.538f,  84.545f }, .duration_to_next_s = 2 },
          { .value = {139.086f,  82.538f,  84.545f }, .duration_to_next_s = 2 },
		  { .value = { 90.000f, 129.149f, 138.414f }, .duration_to_next_s = 2 }
      };
//  const sequence_point_t triangle[] = {
//		/* hip, knee, unused */
//		{ .value = { 90.0f,  90.0f,  90.0f }, .duration_to_next_s = 1 },
//		{ .value = { 120.0f,  120.0f,  120.0f }, .duration_to_next_s = 1 }
//	};
  sequence_init(&leg_seq, triangle, 3u, 3u, selected_mode, true);
  update_active_text(false);
  sequence_stop(&leg_seq);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  last_tick_ms = HAL_GetTick();
  screen_last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  const uint32_t now = HAL_GetTick();
	  if ((now-screen_last_tick) > OLED_PERIOD) {
		  ssd1306_UpdateScreen();
		  screen_last_tick+=OLED_PERIOD;
	  }


	  if ((now - last_tick_ms) > CONTROL_PERIOD) {
		  //Button press set mode and start/stop
		  buttons_update();
		  if (buttons_checkButton(UP) == PUSHED) {
			  selected_mode = (selected_mode + 1) % 3;
			  update_mode_text();
			  sequence_set_mode(&leg_seq, selected_mode);
		  }
		  else if (buttons_checkButton(UP) == RELEASED) {
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  }
		  if (buttons_checkButton(DOWN) == PUSHED) {
			  if (sequence_is_active(&leg_seq)){
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				  update_active_text(false);
				  sequence_stop(&leg_seq);
			  }
			  else {
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				  update_active_text(true);
				  sequence_start(&leg_seq);
				  last_tick_ms = now;
			  }
		  }
		  //Potentiometer set speed
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)raw_adc, 1);
		  speed_scale = 0.5f + ((float)raw_adc[0] / 4095.0f) * 1.5f;
		  speed_scale = roundf(speed_scale * 10.0f) / 10.0f;
		  update_speed_text();
		  //Servo angle advanced by time
		  const float dt = (float)(now - last_tick_ms) * 0.001f;
		  last_tick_ms = now;
		  sequence_update(&leg_seq, dt * speed_scale);
		  //Set new servo angle
		  if (sequence_is_active(&leg_seq)){
			  const float hip_angle = sequence_get_axis(&leg_seq, 0u);
			  const float knee_angle = sequence_get_axis(&leg_seq, 1u);
			  const float ankle_angle = sequence_get_axis(&leg_seq, 2u);
			  __HAL_TIM_SET_COMPARE(&htim3, HIP_CHANNEL, servo_angle_to_us(&hip_cal, hip_angle));
			  __HAL_TIM_SET_COMPARE(&htim3, KNEE_CHANNEL, servo_angle_to_us(&knee_cal, knee_angle));
			  __HAL_TIM_SET_COMPARE(&htim3, ANKLE_CHANNEL, servo_angle_to_us(&ankle_cal, ankle_angle));
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void update_mode_text()
{
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("S-Curve Mode:", Font_6x8, White);
  ssd1306_SetCursor(5, 8);
  if (selected_mode == 0) {
	  ssd1306_WriteString("Linear", Font_6x8, White);
  }
  else if (selected_mode == 1) {
	  ssd1306_WriteString("Cubic", Font_6x8, White);
  }
  else if (selected_mode == 2) {
  	  ssd1306_WriteString("Quintic", Font_6x8, White);
    }
}
void update_speed_text()
{
  char buffer[20];
  ssd1306_SetCursor(0, 20);
  ssd1306_WriteString("Speed Scale:", Font_6x8, White);
  ssd1306_SetCursor(5, 28);
  snprintf(buffer, 20, "%4.2f (%4u/%4u)", speed_scale, raw_adc[0], 4095u);
  ssd1306_WriteString(buffer, Font_6x8, White);
}
void update_active_text(bool active)
{
  ssd1306_SetCursor(70, 55);
  if (active) {
	  ssd1306_WriteString("Running", Font_6x8, White);
  }
  else {
	  ssd1306_WriteString("Stopped", Font_6x8, White);
  }

}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
