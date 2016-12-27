/******************************************************************************
The altHold.c in RaspberryPilot project is placed under the MIT license

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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#include "commonLib.h"
#include "flyControler.h"
#include "vl53l0x.h"
#include "srf02.h"
#include "ms5611.h"
#include "mpu6050.h"
#include "pid.h"
#include "kalmanFilter.h"
#include "motorControl.h"
#include "altHold.h"

#define MODULE_TYPE ALTHOLD_MODULE_MS5611
//#define MODULE_TYPE ALTHOLD_MODULE_SRF02
//#define MODULE_TYPE ALTHOLD_MODULE_VL53L0X
#define ALTHOLD_UPDATE_PERIOD 100000

static float aslRaw = 0.f;
static float altHoldSpeedDeadband = 5.f;
static float altHoldAccSpeedDeadband = 0.5;
static float altHoldSpeed = 0.f;
static float altHoldAccSpeed = 0.f;
static float altHoldAltSpeed = 0.f;
static bool altHoldIsReady = false;
static bool enableAltHold = false;
static bool altholdIsUpdate = false;
static unsigned short maxAlt = 50; 		//cm
static unsigned short targetAlt = 0;
static pthread_t altHoldThreadId;
static pthread_mutex_t altHoldIsUpdateMutex;
static KALMAN_1D_STRUCT altholdKalmanFilterEntry;

static void setAltHoldIsReady(bool v);
static void setMaxAlt(unsigned short v);
static unsigned short getMaxAlt();
static void *altHoldThread(void *arg);
static float getAccWithoutGravity();
static void *altHoldUpdate(void *arg);

/**
 * init althold
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
bool initAltHold() {

	setAltHoldIsReady(false);

	switch (MODULE_TYPE) {

	case ALTHOLD_MODULE_MS5611:
		
		if(!ms5611Init()){
			_DEBUG(DEBUG_NORMAL, "MS5611 Init failed\n");
			return 	false;
		}
		
		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,30.f,1.f,200.f, 0.f);
		setPidDeadBand(&altHoldAltSettings, 30.0);
		setPidDeadBand(&altHoldlSpeedSettings, 30.0);
		setMaxAlt(200); 	//cm

		break;

	case ALTHOLD_MODULE_VL53L0X:

		if (!vl53l0xInit()) {
			_DEBUG(DEBUG_NORMAL, "vl53l0x Init failed\n");
			return false;
		}

		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,10.f,1.f,5.f, 0.f);
		setPidDeadBand(&altHoldAltSettings, 5.0);
		setPidDeadBand(&altHoldlSpeedSettings, 5.0);
		setMaxAlt(150);  		//cm

		break;

	case ALTHOLD_MODULE_SRF02:

		if(!srf02Init()){
			_DEBUG(DEBUG_NORMAL, "SRF02 Init failed\n");
			return 	false;
		}

		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,10.f,1.f,5.f, 0.f);
		setMaxAlt(400);		//cm
		
		break;

	default:

		return false;
	}

	if (pthread_mutex_init(&altHoldIsUpdateMutex, NULL) != 0) {
		_ERROR("(%s-%d) altHoldIsUpdateMutex init failed\n", __func__,
				__LINE__);
		return false;
	}

	if (pthread_create(&altHoldThreadId, NULL, altHoldUpdate, 0)) {
		_DEBUG(DEBUG_NORMAL, "altHold thread create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL, "start altHold thread...\n");
	}
	
	setAltHoldIsReady(true);

	return true;
}

/**
 * get the flag to indicate whether enable althold or not
 *
 * @param
 * 		void
 *
 * @return
 *		altHold is ready or not
 *
 */
bool getEnableAltHold() {
	return enableAltHold;
}

/**
 * set the flag to indicate whether enable althold or not
 *
 * @param v
 * 		altHold is ready or not
 *
 * @return
 *		void
 *
 */
void setEnableAltHold(bool v) {
	enableAltHold = v;
}

/**
 * get the flag to indicate whether althold is ready or not
 *
 * @param
 * 		void
 *
 * @return
 *		altHold is ready or not
 *
 */
bool getAltHoldIsReady() {
	return altHoldIsReady;
}

/**
 * set the flag to indicate whether althold is ready or not
 *
 * @param v
 * 		altHold is ready or not
 *
 * @return
 *		void
 *
 */
void setAltHoldIsReady(bool v) {
	altHoldIsReady = v;
}

/**
 * set the maximum value of altitude
 *
 * @param v
 * 		maximum value of altitude
 *
 * @return
 *		void
 *
 */
void setMaxAlt(unsigned short v) {
	maxAlt = v;
}

/**
 * get the maximum value of altitude
 *
 * @param
 * 		void
 *
 * @return
 *		maximum value of altitude
 *
 */
unsigned short getMaxAlt() {
	return maxAlt;
}

/**
 * calculates acceleration without gravity
 *
 * @param
 * 		void
 *
 * @return
 *		acceleration
 *
 */
float getAccWithoutGravity() {
	return (getXAcc() * getXGravity() + getYAcc() * getYGravity()
			+ getZAcc() * getZGravity() - 1.f);
}

/**
 * get current altitude
 *
 * @param
 * 		void
 *
 * @return
 *		altitude (cm)
 *
 */
float getCurrentAltHoldAltitude() {
	return aslRaw;
}

/**
 * get current speed
 *
 * @param
 * 		void
 *
 * @return
 *		speed (cm/sec)
 *
 */
float getCurrentAltHoldSpeed() {
	return altHoldSpeed;
}

/**
 *  check whether update AltHold info or not
 *
 * @param
 * 		void
 *
 * @return
 *		true or false
 *
 */
bool updateAltHold() {

	bool ret = false;

	pthread_mutex_lock(&altHoldIsUpdateMutex);
	if (altholdIsUpdate) {
		altholdIsUpdate = false;
		ret = true;
	} else {
		ret = false;
	}
	pthread_mutex_unlock(&altHoldIsUpdateMutex);

	return ret;
}

/**
 *  AltHold thread, updates altitude and vertical speed
 *
 * @param arg
 * 		arg
 *
 * @return
 *		pointer
 *
 */
void *altHoldUpdate(void *arg) {

	static float lastAslRaw=0.0;
	static float lastAcl=0.0;
	unsigned short data = 0;
	unsigned long interval=0;
	unsigned long sum=0;
	unsigned int count=0;
	bool result = false;
	struct timeval tv;
	struct timeval tv2;

	while (!getLeaveFlyControlerFlag()&&getAltHoldIsReady()) {

		gettimeofday(&tv,NULL);
	
		if(0!=tv2.tv_usec){

				switch (MODULE_TYPE) {

			        case ALTHOLD_MODULE_MS5611:
					
				result = ms5611GetMeasurementData(&data);
				break;

				case ALTHOLD_MODULE_VL53L0X:

					result = vl53l0xGetMeasurementData(&data);
					break;

				case ALTHOLD_MODULE_SRF02:
				
					result = srf02GetMeasurementData(&data);
					break;
				
				default:
				
					result = false;
				}

				if (result) {
					
					count++;
					sum+=(long)kalmanFilterOneDimCalc((float)data,&altholdKalmanFilterEntry);
					interval=(unsigned long)((tv.tv_sec-tv2.tv_sec)*1000000+(tv.tv_usec-tv2.tv_usec));
					
					if(interval>=ALTHOLD_UPDATE_PERIOD){			
							
							//_DEBUG(DEBUG_NORMAL,"duration=%ld us\n",interval);				
							aslRaw=(float)(sum/(float)count);
							altHoldAltSpeed = deadband((aslRaw - lastAslRaw)/(float)(interval*0.000001),altHoldSpeedDeadband);
							altHoldAccSpeed = deadband(lastAcl * (float)(interval*0.0001),altHoldAccSpeedDeadband);
							altHoldSpeed = altHoldAltSpeed + altHoldAccSpeed ;

							pthread_mutex_lock(&altHoldIsUpdateMutex);
							altholdIsUpdate = true;
							pthread_mutex_unlock(&altHoldIsUpdateMutex);

							lastAslRaw=aslRaw;
							lastAcl=getAccWithoutGravity();
							count=0;
							sum=0;
							tv2.tv_usec=tv.tv_usec;
							tv2.tv_sec=tv.tv_sec;	
	
							_DEBUG_HOVER(DEBUG_HOVER_ALT_SPEED,"(%s-%d) altHoldAltSpeed=%.3f\n", 
									__func__, __LINE__,altHoldAltSpeed);				
							_DEBUG_HOVER(DEBUG_HOVER_ACC_SPEED, "(%s-%d) altHoldAccSpeed=%.3f\n",
									__func__, __LINE__, altHoldAccSpeed);
							_DEBUG_HOVER(DEBUG_HOVER_SPEED, "(%s-%d) altHoldSpeed=%.3f\n",
									__func__, __LINE__, altHoldSpeed);
							_DEBUG_HOVER(DEBUG_HOVER_RAW_ALTITUDE, "(%s-%d) aslRaw=%.3f\n",
									__func__, __LINE__, aslRaw);

					}			
				} else {
					usleep(500);
				}
			
		}else{
			tv2.tv_usec=tv.tv_usec;
			tv2.tv_sec=tv.tv_sec;
		}
	}

	pthread_exit((void *) 1234);

}

/**
 *  convert the percentage of throttle  which is receive from
 *  remote controler into targer altitude
 *
 * @param v
 * 		 percentage of throttle (1 to 100)
 *
 * @return
 *		target altitude
 *
 */
unsigned short convertTargetAltFromeRemoteControler(unsigned short v) {

	targetAlt = (unsigned short)(getMaxAlt()*0.01*(float)v);
	//_DEBUG(DEBUG_NORMAL, "targetAlt=%d\n",targetAlt);
	return targetAlt;
}

/**
 *  get a defeault power level with a altitude, you should make the factor and your Raspberry Pilot match well
 *
 * @param
 * 		 void
 *
 * @return
 *		power level
 *
 */
unsigned short getDefaultPowerLevelWithTargetAlt() {

	unsigned short ret = 0;
	ret= min(getMinPowerLevel()+(unsigned short)((1.0-exp((double)-targetAlt*0.0095))*1300.0),getMaxPowerLeve());
	//_DEBUG(DEBUG_NORMAL, "throttle=%d\n",ret);
	return ret;
}

