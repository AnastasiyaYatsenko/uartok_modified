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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "TMC2209.h"
#include <stdio.h>
#include <cstring>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void printSettingsAndStatus(TMC2209 *tmcd){

	TMC2209::Settings settings = tmcd->getSettings();
	uint8_t version = tmcd->getVersion();
	uint8_t global_status = tmcd->getGlobalStatus();
	uint16_t iopins = tmcd->getIOPins();
	uint16_t mscnt = tmcd->getMicrostepCounter();
	uint32_t drv_status = tmcd->readDrvStatusBytes();
	uint16_t sg_result = tmcd->getStallGuardResult();
	HAL_HalfDuplex_EnableTransmitter(&huart2);
	char str[255];
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n---\r\n\0", strlen("\r\n---\r\n\0"), 30);
	HAL_UART_Transmit(&huart2, (uint8_t*)"getSettings()\r\n\0", strlen("getSettings()\r\n\0"), 30);
	if (!settings.is_communicating) {
		sprintf(str, "\rIS NOT COMMUNICATING\r\n\0");
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
		return;
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n---\r\n\0", strlen("\r\n---\r\n\0"), 30);
	HAL_UART_Transmit(&huart2, (uint8_t*)"getSettings()\r\n\0", strlen("getSettings()\r\n\0"), 30);

	sprintf(str, "\rsettings.is_communicating = %d\r\n\0", settings.is_communicating);  	    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rsettings.is_setup = %d\r\n\0", settings.is_setup);                  	    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rsettings.software_enabled = %d\r\n\0", settings.software_enabled);  	    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rsettings.microsteps_per_step = %d\r\n\0", settings.microsteps_per_step);    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rsettings.inverse_motor_direction_enabled = %d\r\n\0", settings.inverse_motor_direction_enabled);	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rsettings.stealth_chop_enabled = %d\r\n\0", settings.stealth_chop_enabled);	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);

	switch (settings.standstill_mode){
	case TMC2209::NORMAL:
		sprintf(str, "\rsettings.standstill_mode = normal\r\n\0");		   break;
	case TMC2209::FREEWHEELING:
		sprintf(str, "\rsettings.standstill_mode = freewheeling\r\n\0");   break;
	case TMC2209::STRONG_BRAKING:
		sprintf(str, "\rsettings.standstill_mode = strong_braking\r\n\0"); break;
	case TMC2209::BRAKING:
		sprintf(str, "\rsettings.standstill_mode = braking\r\n\0");	       break;
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);

	sprintf(str, "\rGSTAT   = 0x%02x\r\n\0", global_status); HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rIOPins  = 0x%03x\r\n\0", iopins);		 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rVersion = %d\r\n\0",   	 version);		 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rSGRES   = %d\r\n\0",     sg_result);	 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rMSCNT   = %d\r\n\0",     mscnt);		 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\rDRVSTATUS = 0x%08x\r\n\0",drv_status);	 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	sprintf(str, "\r\n\0");				                     HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
	HAL_Delay(10); // after printing
}

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
  DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

    TMC2209 tmcd;
	tmcd.setup(&huart2, 115200, tmcd.SERIAL_ADDRESS_0);
	printSettingsAndStatus(&tmcd);
	HAL_Delay(1000);
	tmcd.setRunCurrent(100);
	tmcd.enable();
	//tmcd.moveAtVelocity(10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*int counter=0;
	int dir = 0;
	int X = 20;
	int v=0,prev_v=0;*/
	bool flag = false;
	tmcd.moveAtVelocity(400); //в районі +-430 його починає жужукать
	while (1) {
		//tmcd.moveAtVelocity(500);
		/*int v=++counter;
		if (counter>=X) v=2*X-counter;
		if (counter>=3*X) v=counter-4*X;
		if (counter>=4*X) counter=0;
		if (v>=0 && prev_v<0) tmcd.enableInverseMotorDirection();
		if (v<0 && prev_v>=0) tmcd.disableInverseMotorDirection();
		tmcd.moveAtVelocity(v>0?v*1000:-v*1000);*/
		tmcd.setMicrostepsPerStepPowerOfTwo(0);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(1);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(2);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(3);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(4);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(5);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(6);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(7);
		HAL_Delay(2000);
		tmcd.setMicrostepsPerStepPowerOfTwo(8);
		HAL_Delay(1000);


		//tmcd.setMicrostepsPerStepPowerOfTwo(8);
		//flag = !flag;
		//HAL_Delay(1000);

		//printSettingsAndStatus(&tmcd);
		//HAL_Delay(100);
		//prev_v = v;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Dir_Pin|En_Pin|Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dir_Pin En_Pin Step_Pin */
  GPIO_InitStruct.Pin = Dir_Pin|En_Pin|Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
