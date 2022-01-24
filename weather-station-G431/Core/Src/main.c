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
#include "eeprom_emul.h"
#include "u8g2/u8g2.h"
#include "bme280/bme280.h"
#include "ccs811/DFRobot_CCS811.h"
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
static void PVD_Config(void);
/* USER CODE BEGIN PFP */

void send_char(char c) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &c, 1, 1000);
}

int __io_putchar(int ch) {
	send_char(ch);
	return ch;
}

#define DEVICE_ADDRESS 0x3C
#define TX_TIMEOUT		100

uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	volatile uint32_t i;
	/* STM32 supports HW SPI, Remove unused cases like U8X8_MSG_DELAY_XXX & U8X8_MSG_GPIO_XXX */
	switch (msg) {

	case U8X8_MSG_DELAY_10MICRO:
		for (i = 0; i < arg_int * 1000; i++) {
			__NOP();
		}

		break;
		//Function which delays 100ns
	case U8X8_MSG_DELAY_100NANO:
		for (i = 0; i < arg_int * 10; i++) {
			__NOP();
		}
		break;
		//Function to define the logic level of the clockline

	case U8X8_MSG_DELAY_I2C:
		//i = 0;
		for (i = 0; i < arg_int * 500; i++)
			;
		break;
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* Insert codes for initialization */
		break;
	case U8X8_MSG_DELAY_MILLI:
		/* ms Delay */
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_CS:
		/* Insert codes for SS pin control */
		//HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_DC:
		/* Insert codes for DC pin control */
		//HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_RESET:
		/* Insert codes for RST pin control */
		//HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, arg_int);
		break;
	default:
		return 0;
	}
	return 1;
}

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buffer[32];
	static uint8_t buf_idx;
	uint8_t *data;

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t*) arg_ptr;
		while (arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		/* add your custom code to init i2c subsystem */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		if (HAL_I2C_Master_Transmit(&hi2c1, (DEVICE_ADDRESS << 1), buffer,
				buf_idx, TX_TIMEOUT) != HAL_OK)
			return 0;
//		HAL_I2C_Master_Transmit_DMA(&hi2c1, (DEVICE_ADDRESS << 1), buffer, buf_idx);
		//HAL_I2C_Mem_Write_DMA(&hi2c1, (DEVICE_ADDRESS << 1), 0x40, 1, &buffer[0], buf_idx);
//		volatile uint32_t i;
//		for (i = 1; i <= 100; i++);
		break;
	default:
		return 0;
	}
	return 1;
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK)
		return -1;
	if (HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10)
			!= HAL_OK)
		return -1;

	return 0;
}

void user_delay_ms(uint32_t period) {
	HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t *buf;
	buf = malloc(len + 1);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);

	if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*) buf, len + 1,
	HAL_MAX_DELAY) != HAL_OK)
		return -1;

	free(buf);
	return 0;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	u8g2_t u8g2;
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

	printf("\r\n");

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

	u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R2,
			u8x8_byte_stm32_hw_i2c, u8x8_stm32_gpio_and_delay);

	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawLine(&u8g2, 0, 0, 100, 100);
	u8g2_SendBuffer(&u8g2);
	u8g2_SetFontDirection(&u8g2, 0);

	/*********EEPROM*******/

	EE_Status ee_status = EE_OK;

	/* Enable and set FLASH Interrupt priority */
	/* FLASH interrupt is used for the purpose of pages clean up under interrupt */
//	HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(FLASH_IRQn);
	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();

	PVD_Config();

	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET) {
//		/* Blink LED_OK (Green) twice at startup */
//		BSP_LED_On(LED_OK);
//		HAL_Delay(100);
//		BSP_LED_Off(LED_OK);
//		HAL_Delay(100);
//		BSP_LED_On(LED_OK);
//		HAL_Delay(100);
//		BSP_LED_Off(LED_OK);

		/* System reset comes from a power-on reset: Forced Erase */
		/* Initialize EEPROM emulation driver (mandatory) */
		ee_status = EE_Init(EE_FORCED_ERASE);
		if (ee_status != EE_OK) {
			Error_Handler();
		}
	} else {
		/* Clear the Standby flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Check and Clear the Wakeup flag */
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF) != RESET) {
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
		}

//		/* Blink LED_OK (Green) upon wakeup */
//		BSP_LED_On(LED_OK);
//		HAL_Delay(100);
//		BSP_LED_Off(LED_OK);

		/* System reset comes from a STANDBY wakeup: Conditional Erase*/
		/* Initialize EEPROM emulation driver (mandatory) */
		ee_status = EE_Init(EE_CONDITIONAL_ERASE);
		if (ee_status != EE_OK) {
			Error_Handler();
		}
	}

	uint32_t number_of_starts = 0;
	uint32_t check;
	__IO uint32_t ErasingOnGoing = 0;

	ee_status = EE_ReadVariable32bits(1, &number_of_starts);

	number_of_starts += 1;

	ee_status = EE_WriteVariable32bits(1, number_of_starts);
	ee_status |= EE_ReadVariable32bits(1, &check);
	if (number_of_starts != check) {
		Error_Handler();
	}

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp_IT();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
		Error_Handler();
	}

	while (ErasingOnGoing == 1) {
	}

	HAL_FLASH_Lock();
	/**********************/

	/****BME280****/

	float temperature;
	float humidity;
	float pressure;

	struct bme280_dev dev;
	struct bme280_data comp_data;
	int8_t rslt;
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(
			BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL
					| BME280_FILTER_SEL, &dev);
	/**************/

	/****CCS811****/
	Init_I2C_CCS811(hi2c1);
	configureCCS811();
	//restore_Baseline();
	//softRest();

	/**************/
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	unsigned long n = 0;
	char tmp[50];
	uint8_t pos_x = 7;
	uint8_t pos_y = 13;
	uint8_t dir_x = 1;
	uint8_t dir_y = 1;
	while (1) {
//		while (dataAvailable() == 0) {
//		}
		readAlgorithmResults();
		unsigned int co2 = getCO2();

		unsigned int baseline = getBaseline();

		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		dev.delay_ms(40);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		if (rslt == BME280_OK) {
			temperature = comp_data.temperature / 100.0; /* °C  */
			humidity = comp_data.humidity / 1024.0; /* %   */
			pressure = comp_data.pressure / 10000.0; /* hPa */

			//memset(line1, 0, sizeof(line1));
			//memset(line1, 0, sizeof(line2));
//			sprintf(line1, "HUMID: %03.1f ", humidity);
//			sprintf(line2, "HUMID: %03.1f", temperature);

		}

		HAL_Delay(10);
		u8g2_ClearBuffer(&u8g2);
		//u8g2_ClearDisplay(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_fur11_tf);
		sprintf(tmp, "%u %u, %x", number_of_starts, co2, baseline);
		u8g2_DrawStr(&u8g2, 0, 20, tmp);
		sprintf(tmp, "H: %03.1f%%", humidity);
		u8g2_DrawStr(&u8g2, 0, 40, tmp);
		sprintf(tmp, "T: %03.1f", temperature);
		u8g2_DrawStr(&u8g2, 0, 60, tmp);
		//u8g2_SetDrawColor(&u8g2, 0);
		u8g2_DrawBox(&u8g2, pos_x, pos_y, 5, 5);
		//u8g2_SetDrawColor(&u8g2, 0);
		//u8g2_DrawBox(&u8g2, pos_x, pos_y, 3, 3);
		//u8g2_SetDrawColor(&u8g2, 1);
		pos_x += dir_x;
		pos_y += dir_y;
			// TODO DEBUG level 0 and -02
		if (pos_x >= 123 || pos_x <= 0) {
			dir_x = -dir_x;
		}
		if (pos_y >= 59 || pos_y <= 0) {
			dir_y = -dir_y;
		}

		//u8g2_DrawStr(&u8g2, 0, n % 88, tmp);
		u8g2_SendBuffer(&u8g2);
		n += 1;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 25;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x009034B6;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

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
void Error_Handler(void) {
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
