/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED_F407.h"
#include "OLED.h"
#include "IST8310.h"
#include "BMI088.h"
#include "IMU_C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float accdata[3] = {0};
float gyrodata[3] = {0};
float magdata[3] = {0};
float g_roll;
float g_pitch;
float g_yaw;
uint8_t g_oled_page = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
//{
////	 LED_ON(LED_R);
//	HAL_WWDG_Refresh(&hwwdg);
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t count = 0;
	if (htim->Instance == TIM2)
	{
		count++;
		count %= 1000;
		if(count % 5 == 0)
		{
//			HAL_WWDG_Refresh(&hwwdg);
			IMU_Data_Fusion_Mahony(0.005, &g_roll, &g_pitch, &g_yaw);
			IMU_Temperature_Compensate();

		}
		if(count % 500 == 0)
		{
			LED_TOG(LED_R);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == KEY_Pin)
	{
		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
		{
			g_oled_page++;
			g_oled_page %= 5;
			LED_TOG(LED_B);
			OLED_Clear();
			OLED_ShowNum(1, 14, g_oled_page, 1);
		}
	}
}

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
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  uint8_t IMU_error = 0;
  OLED_Init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  IMU_error = IMU_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	 HAL_Delay(50);
//	 for(uint8_t i = 0; i < 3; i++)
//	 {
//		  OLED_Showfloat(2 + i, 1, accdata[i], 4, 4);
//	 }
	  HAL_Delay(50);
	  switch (g_oled_page)
	  {
	  case 0:
		  OLED_ShowString(1, 1, "IMU DEMO");
		  OLED_ShowString(2, 1, "ERROR:");OLED_ShowBinNum(2, 7, IMU_error, 8);
		  OLED_ShowString(3, 1, "TEMP:"); OLED_Showfloat(3, 6, BMI088_Get_Temperature(), 4, 5);
		  break;
	  case 1:
		  OLED_ShowString(1, 1, "Angle");
		  OLED_Showfloat(2, 4, g_roll , 4, 4);
		  OLED_Showfloat(3, 4, g_pitch, 4, 4);
		  OLED_Showfloat(4, 4, g_yaw  , 4, 4);
		  break;
	  case 2:
		  BMI088_Getdata_Acc(accdata);
		  OLED_ShowString(1, 1, "A-C-C");
//		  OLED_ShowHexNum(1, 12, BMI088_Acc_ReadID(), 2);
		  OLED_Showfloat(2, 1, accdata[0], 4, 4);
		  OLED_Showfloat(3, 1, accdata[1], 4, 4);
		  OLED_Showfloat(4, 1, accdata[2], 4, 4);
		  break;
	  case 3:
		  BMI088_Getdata_Gyro(gyrodata);
		  OLED_ShowString(1, 1, "G-Y-R-O");
//		  OLED_ShowHexNum(1, 12, BMI088_Gyro_ReadID(), 2);
		  OLED_Showfloat(2, 1, gyrodata[0], 4, 5);
		  OLED_Showfloat(3, 1, gyrodata[1], 4, 5);
		  OLED_Showfloat(4, 1, gyrodata[2], 4, 5);
		  break;
	  case 4:
		  IST8310_Getdata_Mag(magdata);
		  OLED_ShowString(1, 1, "M-A-G");
		  OLED_ShowHexNum(1, 12, IST8310_ReadID(), 2);
		  OLED_Showfloat(2, 1, magdata[0], 4, 5);
		  OLED_Showfloat(3, 1, magdata[1], 4, 5);
		  OLED_Showfloat(4, 1, magdata[2], 4, 5);
		  break;
	  default: break;
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
