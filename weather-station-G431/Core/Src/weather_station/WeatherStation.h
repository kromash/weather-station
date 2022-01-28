/*
 * WeatherStation.h
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#ifndef SRC_WEATHER_STATION_WEATHERSTATION_H_
#define SRC_WEATHER_STATION_WEATHERSTATION_H_

#include "stm32g4xx_hal.h"
#include "devices/BME280.h"
#include "devices/CCS811.h"
#include "devices/Display.h"
#include <cstdlib> //malloc
#include <cstring> //memcpy

namespace I2C {
extern I2C_HandleTypeDef hi2c1;
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
}
class WeatherStation {
private:
	float temperature, humidity, pressure;

	I2C_HandleTypeDef* i2c;
	Display display;
	CCS811 ccs811;
	BME280 bme280;

	void init_bme280();
	void init_ccs811();

	public:
	WeatherStation(I2C_HandleTypeDef* i2c, Display display);

	void run();
	virtual ~WeatherStation();

};

#endif /* SRC_WEATHER_STATION_WEATHERSTATION_H_ */
