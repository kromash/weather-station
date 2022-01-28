/*
 * BME280.h
 *
 *  Created on: Jan 24, 2022
 *      Author: kromash
 */

#ifndef SRC_WEATHER_STATION_BME280_H_
#define SRC_WEATHER_STATION_BME280_H_


class Data {
	float temperature;
	float humidity;
	float pressure;
};

class BME280 {
public:
	BME280();
	virtual ~BME280();
};

#endif /* SRC_WEATHER_STATION_BME280_H_ */
