/*
 * Display.h
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#ifndef SRC_WEATHER_STATION_DISPLAY_H_
#define SRC_WEATHER_STATION_DISPLAY_H_

#include <stm32g4xx_hal.h>
#include <sys/_stdint.h>
#include <stdio.h>
#include "../../u8g2/u8g2.h"
#define TX_TIMEOUT		100

namespace display {
extern I2C_HandleTypeDef *hi2c;
extern uint8_t display_i2c_address;

void setup_display(I2C_HandleTypeDef *hi2c, uint8_t display_i2c_address);
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
}
struct WeatherData {
	float temperature, humidity, pressure;
	unsigned int co2, baseline;
};

class Display {
private:
	WeatherData weather_data;
	int point_x, point_y, dir_x, dir_y;
	int width, heigth;
	int point_size=16;
	u8g2_t u8g2;

public:

	Display(I2C_HandleTypeDef *hi2c, int width, int heigth);

	void update_display();
	void update_data(WeatherData _weather_data);
	virtual ~Display();

};

#endif /* SRC_WEATHER_STATION_DISPLAY_H_ */
