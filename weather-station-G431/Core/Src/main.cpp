/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "weather_station/WeatherStation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWR_FLAG_WUF PWR_FLAG_WUF2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void PVD_Config(void);
#ifdef __cplusplus
extern "C" {
#endif
void send_char(char c) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &c, 1, 1000);
}

int __io_putchar(int ch) {
	send_char(ch);
	return ch;
}
#ifdef __cplusplus
}
#endif

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

//
//	u8g2_DrawLine(&u8g2, 50, 50, 100, 100);
//	u8g2_SendBuffer(&u8g2);
	char uart2Data[24] = "Connected to UART Two\r\n";
	/*
	 * Output to uart2
	 * use screen or putty or whatever terminal software
	 * 8N1 115200
	 */
	HAL_UART_Transmit(&huart2, (uint8_t*) &uart2Data, sizeof(uart2Data),
			0xFFFF);

	//printf("\r\n");

	printf("Scanning I2C bus:\r\n");
	HAL_StatusTypeDef result;
	uint8_t i;
	for (i = 1; i < 128; i++) {
		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 2, 2);
		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
				{
			printf("."); // No ACK received at that address
		}
		if (result == HAL_OK) {
			printf("0x%X", i); // Received an ACK at that address
		}
	}
	printf("\r\n");

	/*********EEPROM*******/

	PVD_Config();

//	EE_Status ee_status = EE_OK;
//
//	/* Enable and set FLASH Interrupt priority */
//	/* FLASH interrupt is used for the purpose of pages clean up under interrupt */
////	HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
////	HAL_NVIC_EnableIRQ(FLASH_IRQn);
//	/* Unlock the Flash Program Erase controller */
//	HAL_FLASH_Unlock();
//
//
//	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET) {
//
//		/* System reset comes from a power-on reset: Forced Erase */
//		/* Initialize EEPROM emulation driver (mandatory) */
//		ee_status = EE_Init(EE_FORCED_ERASE);
//		if (ee_status != EE_OK) {
//			Error_Handler();
//		}
//	} else {
//		/* Clear the Standby flag */
//		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
//
//		/* Check and Clear the Wakeup flag */
//		if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF) != RESET) {
//			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
//		}
//
//
//		/* System reset comes from a STANDBY wakeup: Conditional Erase*/
//		/* Initialize EEPROM emulation driver (mandatory) */
//		ee_status = EE_Init(EE_CONDITIONAL_ERASE);
//		if (ee_status != EE_OK) {
//			Error_Handler();
//		}
//	}
//
//	uint32_t number_of_starts = 0;
//	uint32_t check;
//	__IO uint32_t ErasingOnGoing = 0;
//
//	ee_status = EE_ReadVariable32bits(1, &number_of_starts);
//
//	number_of_starts += 1;
//
//	ee_status = EE_WriteVariable32bits(1, number_of_starts);
//	ee_status = static_cast<EE_Status>(EE_ReadVariable32bits(1, &check));
//	if (number_of_starts != check) {
//		Error_Handler();
//	}
//
//	/* Start cleanup IT mode, if cleanup is needed */
//	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
//		ErasingOnGoing = 1;
//		ee_status = static_cast<EE_Status>(ee_status | EE_CleanUp_IT());
//	}
//	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
//		Error_Handler();
//	}
//
//	while (ErasingOnGoing == 1) {
//	}
//
//	HAL_FLASH_Lock();

	/**********************/

	/*WTHSTATION*/
	Display display = Display(&hi2c1, 128, 64);
	WeatherStation weather_station = WeatherStation(&hi2c1, display);
	weather_station.run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Programmable Voltage Detector (PVD) Configuration
 *         PVD set to level 6 for a threshold around 2.9V.
 * @param  None
 * @retval None
 */
static void PVD_Config(void) {
	PWR_PVDTypeDef sConfigPVD;
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
	if (HAL_PWR_ConfigPVD(&sConfigPVD) != HAL_OK) {
		Error_Handler();
	}

	/* Enable PVD */
	HAL_PWR_EnablePVD();

	/* Enable and set PVD Interrupt priority */
	HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
