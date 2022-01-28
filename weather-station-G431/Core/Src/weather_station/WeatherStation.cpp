/*
 * WeatherStation.cpp
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#include "WeatherStation.h"
#include "../bme280/bme280.h"

namespace I2C {
I2C_HandleTypeDef hi2c1;
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
	// TODO new?
	buf = (int8_t *)malloc(len + 1);
	buf[0] = reg_addr;
	std::memcpy(buf + 1, data, len);

	if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*) buf, len + 1,
	HAL_MAX_DELAY) != HAL_OK)
		return -1;

	free(buf);
	return 0;
}
}

WeatherStation::WeatherStation(I2C_HandleTypeDef *i2c, Display display) :
		i2c(i2c), display(display) {
	init_bme280();
	init_ccs811();
}

WeatherStation::~WeatherStation() {
	// TODO Auto-generated destructor stub
}
void WeatherStation::init_bme280() {
	struct bme280_dev dev;
	struct bme280_data comp_data;
	int8_t rslt;
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = I2C::user_i2c_read;
	dev.write = I2C::user_i2c_write;
	dev.delay_ms = I2C::user_delay_ms;

	rslt = bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(
			BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL
					| BME280_FILTER_SEL, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev); //BME280_FORCED_MODE
	dev.delay_ms(40);
}

void WeatherStation::init_ccs811() {

}

void WeatherStation::run() {
	while (1) {
		HAL_Delay(1);
		display.update_data();
		display.update_display();
	}
}



