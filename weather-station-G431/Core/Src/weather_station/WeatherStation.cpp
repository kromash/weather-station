/*
 * WeatherStation.cpp
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#include "WeatherStation.h"

namespace I2C {
I2C_HandleTypeDef hi2c1;
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
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

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
	int8_t *buf;
	// TODO new?
	buf = (int8_t*) malloc(len + 1);
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
	I2C::hi2c1 = *i2c;
	init_bme280();
	init_ccs811();
	read_eeprom();
}

WeatherStation::~WeatherStation() {
	// TODO Auto-generated destructor stub
}
void WeatherStation::init_bme280() {

	int8_t rslt;
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = I2C::user_i2c_read;
	dev.write = I2C::user_i2c_write;
	dev.delay_ms = I2C::user_delay_ms;

	rslt = bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_4X;
	dev.settings.standby_time = BME280_STANDBY_TIME_500_MS;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(
	BME280_ALL_SETTINGS_SEL, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev); //BME280_FORCED_MODE
	dev.delay_ms(40);
}

void WeatherStation::init_ccs811() {
	Init_I2C_CCS811(I2C::hi2c1);
	configureCCS811();
	restore_Baseline();
}

void WeatherStation::read_eeprom() {
	EE_Status ee_status = EE_OK;

	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();

	//PVD_Config();

	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET) {

		/* System reset comes from a power-on reset: Forced Erase */
		/* Initialize EEPROM emulation driver (mandatory) */
		ee_status = EE_Init(EE_FORCED_ERASE);
		if (ee_status != EE_OK) {
			handle_error();
		}
	} else {
		/* Clear the Standby flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Check and Clear the Wakeup flag */
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != RESET) {
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
		}

		/* System reset comes from a STANDBY wakeup: Conditional Erase*/
		/* Initialize EEPROM emulation driver (mandatory) */
		ee_status = EE_Init(EE_CONDITIONAL_ERASE);
		if (ee_status != EE_OK) {
			handle_error();
		}
	}

	uint32_t check;
	__IO uint32_t ErasingOnGoing = 0;
	number_of_starts = 0;
	ee_status = EE_ReadVariable32bits(1, &number_of_starts);

	number_of_starts += 1;

	ee_status = EE_WriteVariable32bits(1, number_of_starts);
	ee_status = static_cast<EE_Status>(EE_ReadVariable32bits(1, &check));
	if (number_of_starts != check) {
		handle_error();
	}

	/* Start cleanup IT mode, if cleanup is needed */
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
		ErasingOnGoing = 1;
		ee_status = static_cast<EE_Status>(ee_status | EE_CleanUp_IT());
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
		handle_error();
	}

	while (ErasingOnGoing == 1) {
	}

	HAL_FLASH_Lock();
}

void WeatherStation::handle_error() {
	__disable_irq();
	while (1) {
	}
}

void WeatherStation::run() {
	WeatherData weather_data;
	int8_t rslt;
	restore_Baseline();
	while (1) {
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		if (rslt == BME280_OK) {
			weather_data.temperature = comp_data.temperature / 100.0; /* °C  */
			weather_data.temperature -= 4; // TODO compensate
			weather_data.humidity = comp_data.humidity / 1024.0; /* %   */
			weather_data.pressure = comp_data.pressure / 10000.0; /* hPa */

			setEnvironmentalData(weather_data.humidity,
					weather_data.temperature);
		}
		readAlgorithmResults();
		weather_data.co2 = getCO2();

		weather_data.baseline = getBaseline();

		//weather_data.temperature=30;
		display.update_data(weather_data, number_of_starts);
		display.update_display();
		HAL_Delay(500);

	}
}

