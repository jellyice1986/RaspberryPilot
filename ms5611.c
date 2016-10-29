#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "commonLib.h"
#include "i2c.h"
#include "ms5611.h"


#define MS5611_ADDR_CSB_HIGH    0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR_CSB_LOW     0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
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
#define CONST_SEA_PRESSURE 1022.2f //Hsinchu city
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor

void readCalibrationDataFromProm();
void sendPressCmdD1();
double readPress();
void sendTempCmdD2();
double readTemp();
char getPressD1Cmd();
char getTempD2Cmd();
void getDelay();
void resetMs5611();

static unsigned short osr;
static unsigned short calibration[6];
static double deltaTemp;
static double temperature;


	
bool ms5611Init(){

	osr=4096;
	deltaTemp=0;
	temperature=0;
	resetMs5611();
	usleep(10000);
	readCalibrationDataFromProm();
	return true;
}

void resetMs5611(){
	writeByte(MS5611_ADDR_CSB_LOW, MS5611_RESET, true);
}
    
double getAltitude(){
	double  altitude=0.0;
	double tmp=0;
	double press=0;
	static int count=0;
	sendTempCmdD2();
	getDelay();
	tmp=readTemp();
	sendPressCmdD1();
	getDelay();
	press=readPress();
	altitude =((powf((CONST_SEA_PRESSURE / press), CONST_PF) - 1.0f) * (tmp + 273.15f)) / 0.0065f;
	//printf("altitude=%5.5f, mbar=%f, temp=%5.5f\n",altitude,press,(float)temperature/100.f);
	return altitude;
}
/**
* calibration[0]: Pressure sensitivity | SENST1
* calibration[1]: Pressure offset | OFFT1
* calibration[2]: Temperature coefficient of pressure sensitivity | TCS
* calibration[3]: Temperature coefficient of pressure offset | TCO
* calibration[4]: Reference temperature | TREF
* calibration[5]: Temperature coefficient of the temperature | TEMPSENS
*/
void readCalibrationDataFromProm(){
	unsigned char data[2];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR,2,data);
	calibration[0]=((unsigned short)data[0] << 8) | data[1];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR+2,2,data);
	calibration[1]=((unsigned short)data[0] << 8) | data[1];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR+4,2,data);
	calibration[2]=((unsigned short)data[0] << 8) | data[1];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR+6,2,data);
	calibration[3]=((unsigned short)data[0] << 8) | data[1];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR+8,2,data);
	calibration[4]=((unsigned short)data[0] << 8) | data[1];
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_CALIB_ADDR+10,2,data);
	calibration[5]=((unsigned short)data[0] << 8) | data[1];
	printf("%d %d %d %d %d %d\n",calibration[0],calibration[1],calibration[2],calibration[3],calibration[4],calibration[5]);
}

void sendPressCmdD1(){
	writeByte(MS5611_ADDR_CSB_LOW, getPressD1Cmd(), true);
}

void sendTempCmdD2(){
	writeByte(MS5611_ADDR_CSB_LOW, getTempD2Cmd(), true);
}

double readPress(){
	unsigned char data[3];
	double offset=0;
	double sens=0;
	double offset2=0;
	double sens2=0;
	unsigned long rawPressure=0;
	double pressureUnscaled=0;



	
		readBytes(MS5611_ADDR_CSB_LOW, MS5611_ADC_READ,3,data);
		rawPressure = (data[0] << 16) | (data[1] << 8) | (data[2] << 0);
		sens=calibration[0]*(float)pow(2,15)+deltaTemp*calibration[2]/(float)pow(2,8);
		offset =calibration[1]*(float)pow(2,16)+deltaTemp*calibration[3]/(float)pow(2,7);
		
		
		// second order temperature compensation
		if (temperature < 2000.f) {
			offset2 = 5.f * ((temperature - 2000.f) * (temperature - 2000.f))/2.f;
			sens2 = 5.f * ((temperature - 2000.f) * (temperature - 2000.f))/4.f;

			if (temperature < -1500.f) {
				offset2 = offset2+7 * (temperature + 1500.f) * (temperature + 1500.f);
				sens2 = sens2+11 * ((temperature + 1500.f) * (temperature + 1500.f))/2.f;
			}
		}
		pressureUnscaled=(((rawPressure*(sens-sens2))/(float)pow(2,21)-(offset-offset2))/(float)pow(2,15))/100.f;
		

	//printf("rawPressure=%ld, offset=%f, offset2=%f, sens=%f, sens2=%f, pressureUnscaled=%f\n",rawPressure,offset,offset2,sens,sens2,pressureUnscaled);
		return  pressureUnscaled;//mbar 
	
}

double readTemp(){
	unsigned char data[3];
	unsigned long  rawTemperature=0;
	
	
	readBytes(MS5611_ADDR_CSB_LOW, MS5611_ADC_READ,3,data);
	rawTemperature = (data[0] << 16) | (data[1] << 8) | (data[2]<<0);
	deltaTemp=rawTemperature-calibration[4]*(float)pow(2,8);
	temperature=(2000.f+(deltaTemp*calibration[5])/(float)pow(2,23));

	// second order temperature compensation
	if (temperature < 2000.f)
		temperature =temperature- ((deltaTemp * deltaTemp)/(float)pow(2,31));
	
	//printf("deltaTemp=%f, temperature=%f, temperatureUnscaled=%f\n",deltaTemp,temperature,temperatureUnscaled);
	
	return ((double) temperature) / 100.0f;
}



char getPressD1Cmd(){
	switch(osr) {
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

char getTempD2Cmd(){
	switch(osr) {
		
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

void getDelay()
{
	switch(osr) {
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
