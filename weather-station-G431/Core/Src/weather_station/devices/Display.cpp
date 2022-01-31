/*
 * Display.cpp
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#include "Display.h"

namespace display {
I2C_HandleTypeDef *hi2c;
uint8_t display_i2c_address;

void setup_display(I2C_HandleTypeDef *hi2c,
		uint8_t display_i2c_address) {
	display::hi2c = hi2c;
	display::display_i2c_address = display_i2c_address;
}

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg,
		uint8_t arg_int, void *arg_ptr) {
	/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
//	using display::hi2c;
//	using display::display_i2c_address;

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
		if (HAL_I2C_Master_Transmit(hi2c,
				(display_i2c_address << 1), buffer, buf_idx,
				TX_TIMEOUT) != HAL_OK)
			return 0;
		break;
	default:
		return 0;
	}
	return 1;
}

uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg,
		uint8_t arg_int, void *arg_ptr) {
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

}

Display::Display(I2C_HandleTypeDef *hi2c, int width, int heigth) :point_x(1), point_y(5),
		dir_x(1), dir_y(1), width(width), heigth(heigth) {

	//display::hi2c = hi2c;
	//display::display_i2c_address = 0x3C;

	display::setup_display(hi2c, 0x3C);
	u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R2,
			display::u8x8_byte_stm32_hw_i2c,
			display::u8x8_stm32_gpio_and_delay);

	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawLine(&u8g2, 0, 0, 50, 100);
	u8g2_SendBuffer(&u8g2);
	u8g2_SetFontDirection(&u8g2, 0);

	u8g2_SetFont(&u8g2, u8g2_font_fur11_tf);

}

void Display::update_display() {
	u8g2_ClearBuffer(&u8g2);

	//TODO u8g2_DrawBox(&u8g2, point_x, point_y, point_size, point_size);
	char tmp[50];
	snprintf(tmp, 50, "%u, %x", weather_data.co2, weather_data.baseline);
	u8g2_DrawStr(&u8g2, 0, 20, tmp);
	snprintf(tmp, 50, "H: %03.1f%%", weather_data.humidity);
	u8g2_DrawStr(&u8g2, 0, 40, tmp);
	snprintf(tmp, 50, "T: %03.1f P: %03.1f hPA", weather_data.temperature, weather_data.pressure);
	u8g2_DrawStr(&u8g2, 0, 60, tmp);


	u8g2_SendBuffer(&u8g2);

	point_x += dir_x;
	point_y += dir_y;
	if (point_x >= width - point_size || point_x <= 0) {
		dir_x = -dir_x;
	}
	if (point_y >= heigth - point_size || point_y <= 0) {
		dir_y = -dir_y;
	}
}

void Display::update_data(WeatherData _weather_data) {
	weather_data = _weather_data;
}




Display::~Display() {
	// TODO Auto-generated destructor stub
}

