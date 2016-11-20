/******************************************************************************
The ms5611.c in RaspberryPilot project is placed under the MIT license

Copyright (c) 2016 jellyice1986 (Tung-Cheng Wu)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "commonLib.h"
#include "i2c.h"
#include "ms5611.h"

#define MS5611_ADDR_CSB_HIGH    0x76   
#define MS5611_ADDR_CSB_LOW     0x77
#define MS5611_RESET            0x1E
#define MS5611_CALIB_ADDR       0xA2
#define MS5611_CALIB_LEN        12
#define MS5611_ADC_READ         0x00
#define MS5611_PRES_D1_OSR_256  0x40
#define MS5611_PRES_D1_OSR_512  0x42
#define MS5611_PRES_D1_OSR_1024 0x44
#define MS5611_PRES_D1_OSR_2048 0x46
#define MS5611_PRES_D1_OSR_4096 0x48
#define MS5611_TEMP_D2_OSR_256  0x50
#define MS5611_TEMP_D2_OSR_512  0x52
#define MS5611_TEMP_D2_OSR_1024 0x54
#define MS5611_TEMP_D2_OSR_2048 0x56
#define MS5611_TEMP_D2_OSR_4096 0x58
#define MS5611_ADC_MSB          0xF6
#define CONST_SEA_PRESSURE 		1016.3f //Hsinchu city
#define CONST_PF 				0.1902630958f //(1/5.25588f)
#define CONST_PF2 				153.8461538461538f //(1/0.0065)

void readCalibrationDataFromProm();
void sendPressCmdD1();
float readPress();
void sendTempCmdD2();
float readTemp();
char getPressD1Cmd();
char getTempD2Cmd();
void getDelay();
void resetMs5611();
float kalmanFilterOneDim(float inData);

static unsigned short osr;
static unsigned short calibration[6];
static float deltaTemp;   //dt
static float temperature;

/**
 * Init MS5611
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool ms5611Init() {

	if (checkI2cDeviceIsExist(MS5611_ADDR_CSB_LOW)) {
		_DEBUG(DEBUG_NORMAL, "(%s-%d) MS5611 exist\n", __func__, __LINE__);
	} else {
		_ERROR("(%s-%d) MS5611 dowsn't exist\n", __func__, __LINE__);
		return false;
	}

	osr = 4096;
	deltaTemp = 0;
	temperature = 0;
	resetMs5611();
	usleep(20000);
	readCalibrationDataFromProm();

	return true;
}

/**
 * get temperature and pressure from MS5611 and calculate attitude
 *
 * @param cm
 * 		altitude
 *
 * @return
 *		bool
 *
 */
bool ms5611GetMeasurementData(unsigned short *cm) {

	float tmp = 0;
	float press = 0;

	//send cmd D2 and read tmp
	sendTempCmdD2();
	getDelay();
	tmp = readTemp();

	//send cmd D1 and read press
	sendPressCmdD1();
	getDelay();
	press = readPress();

	//altitude = ( ( (Sea-level pressure/Atmospheric pressure)^ (1/5.257)-1 ) * (temperature+273.15))/0.0065
	*cm = (unsigned short)(((powf((CONST_SEA_PRESSURE / press), CONST_PF) - 1.0f)
			* (tmp + 273.15f)) * CONST_PF2 * 100.f);
	//_DEBUG(DEBUG_NORMAL, "ms5611 rawAltitude=%d, mbar=%.2f, temp=%.2f\n", *cm,press, tmp);
	
	return true;
}

/**
 * reset MS5611
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void resetMs5611() {
	writeByte(MS5611_ADDR_CSB_LOW, MS5611_RESET, true);
}

/**
 * read calibration data from the PROM in MS5611:
 * 	calibration[0]: Pressure sensitivity | SENST1
 * 	calibration[1]: Pressure offset | OFFT1
 * 	calibration[2]: Temperature coefficient of pressure sensitivity | TCS
 * 	calibration[3]: Temperature coefficient of pressure offset | TCO
 * 	calibration[4]: Reference temperature | TREF
 * 	calibration[5]: Temperature coefficient of the temperature | TEMPSENS
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void readCalibrationDataFromProm() {

	unsigned char data[2];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR, 2, data);
	calibration[0] = ((unsigned short) data[0] << 8) | data[1];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR + 2, 2, data);
	calibration[1] = ((unsigned short) data[0] << 8) | data[1];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR + 4, 2, data);
	calibration[2] = ((unsigned short) data[0] << 8) | data[1];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR + 6, 2, data);
	calibration[3] = ((unsigned short) data[0] << 8) | data[1];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR + 8, 2, data);
	calibration[4] = ((unsigned short) data[0] << 8) | data[1];
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR + 10, 2, data);
	calibration[5] = ((unsigned short) data[0] << 8) | data[1];

	_DEBUG(DEBUG_NORMAL, "Ms5611 calibbration data: %d %d %d %d %d %d\n",
			calibration[0], calibration[1], calibration[2], calibration[3],
			calibration[4], calibration[5]);
}

/**
 * send cmd D1 befor read pressure data:
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void sendPressCmdD1() {
	writeByte(MS5611_ADDR_CSB_LOW, getPressD1Cmd(), true);
}

/**
 * send cmd D2 befor read pressure data:
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void sendTempCmdD2() {
	writeByte(MS5611_ADDR_CSB_LOW, getTempD2Cmd(), true);
}

/**
 * read pressure after send cmd D1
 *
 * @param
 * 		void
 *
 * @return
 *		pressure (mbar or hbar)
 *
 */
float readPress() {

	unsigned char data[3];
	float offset = 0.f;
	float sens = 0.f;
	float offset2 = 0.f;
	float sens2 = 0.f;
	unsigned long rawPressure = 0;
	float pressureUnscaled = 0.f;

	readBytes(MS5611_ADDR_CSB_LOW, MS5611_ADC_READ, 3, data);
	rawPressure = (data[0] << 16) | (data[1] << 8) | (data[2] << 0);

	//SENS = C1 * 2^15 + (C3 * dT) / 2^8
	sens = (float) calibration[0] * 32768.f
			+ (float) calibration[2] * deltaTemp * 0.00390625f;
	//OFF = C2 * 2^16 + (C4* dT) / 2^7
	offset = (float) calibration[1] * 65536.f
			+ (float) calibration[3] * deltaTemp * 0.0078125f;

	// second order temperature compensation
	if (temperature < 2000.f) {

		//OFF2 = 5 *((TEMP ¡V 2000)^2 )/ 2^1
		offset2 = 2.5f * (temperature - 2000.f) * (temperature - 2000.f);
		//SENS2 = 5 *(TEMP ¡V 2000)^2/ 2^2
		sens2 = 1.25f * (temperature - 2000.f) * (temperature - 2000.f);

		if (temperature < -1500.f) {
			//OFF2 = OFF2 + 7 *(TEMP + 1500)^2
			offset2 = offset2
					+ 7.f * (temperature + 1500.f) * (temperature + 1500.f);
			//SENS2 = SENS2 + (11 * (TEMP + 1500)^2)/ 2^1
			sens2 = sens2
					+ (5.5f * (temperature + 1500.f) * (temperature + 1500.f));
		}
	}

	//OFF = OFF - OFF2
	//SENS = SENS - SENS2
	//P = (D1 * SENS / 2^21 - OFF) / 2^15
	pressureUnscaled =
			(((rawPressure * (sens - sens2)) * 0.000000476837158203125f
					- (offset - offset2)) * 0.000030517578125) * 0.01f;

	return pressureUnscaled;

}

/**
 * read pressure after send cmd D1
 *
 * @param
 * 		void
 *
 * @return
 *		temperature (Celsius)
 *
 */
float readTemp() {

	unsigned char data[3];
	unsigned int rawTemperature = 0;
	float tempOutput = 0.f;

	readBytes(MS5611_ADDR_CSB_LOW, MS5611_ADC_READ, 3, data);
	rawTemperature = (data[0] << 16) | (data[1] << 8) | (data[2] << 0);

	//dt = D2-C5*2^8
	deltaTemp = (float) rawTemperature - (float) calibration[4] * 256.f;
	//TEMP = 2000 + (dT *C6 / (2^23)
	//temperature will be used to calculate pressure
	tempOutput = temperature = (2000.f
			+ (deltaTemp * calibration[5]) * 0.00000011920928955078125f);

	// second order temperature compensation
	if (temperature < 2000.f) {
		//T2 = dT^2 / 2^31
		//TEMP = TEMP - T2
		tempOutput = temperature
				- (deltaTemp * deltaTemp * 0.0000000004656612873077392578125f);
	}

	return (tempOutput) * 0.01f;
}

/**
 * get D1 command by OSR setting
 *
 * @param
 * 		void
 *
 * @return
 *		command
 *
 */
char getPressD1Cmd() {
	switch (osr) {
	case 256:
		return MS5611_PRES_D1_OSR_256;
	case 512:
		return MS5611_PRES_D1_OSR_512;
	case 1024:
		return MS5611_PRES_D1_OSR_1024;
	case 2048:
		return MS5611_PRES_D1_OSR_2048;
	case 4096:
		return MS5611_PRES_D1_OSR_4096;
	default:
		break;
	}
	return MS5611_PRES_D1_OSR_4096;
}

/**
 * get D2 command by OSR setting
 *
 * @param
 * 		void
 *
 * @return
 *		command
 *
 */
char getTempD2Cmd() {

	switch (osr) {
	case 256:
		return MS5611_TEMP_D2_OSR_256;
	case 512:
		return MS5611_TEMP_D2_OSR_512;
	case 1024:
		return MS5611_TEMP_D2_OSR_1024;
	case 2048:
		return MS5611_TEMP_D2_OSR_2048;
	case 4096:
		return MS5611_TEMP_D2_OSR_4096;
	default:
		break;
	}
	return MS5611_TEMP_D2_OSR_4096;
}

/**
 * get delay by OSR setting
 *
 * @param
 * 		void
 *
 * @return
 *		command
 *
 */
void getDelay() {

	switch (osr) {
	case 256:
		usleep(2000);
		break;
	case 512:
		usleep(2000);
		break;
	case 1024:
		usleep(3000);
		break;
	case 2048:
		usleep(5000);
		break;
	case 4096:
		usleep(10000);
		break;
	default:
		break;
	}
}

