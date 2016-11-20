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
#include <sys/time.h>
#include <pthread.h>
#include "commonLib.h"
#include "flyControler.h"
#include "vl53l0x.h"
#include "srf02.h"
#include "ms5611.h"
#include "mpu6050.h"
#include "altHold.h"
#include "kalmanFilter.h"
#include "motorControl.h"

#define ALTHOLD_CHECK_CYCLE_TIME 0
#define MODULE_TYPE ALTHOLD_MODULE_MS5611
//#define MODULE_TYPE ALTHOLD_MODULE_SRF02
//#define MODULE_TYPE ALTHOLD_MODULE_VL53L0X

static float aslRaw = 0.f;
static float asl = 0.f;
static float aslLong = 0.f;
static float aslAlpha = 0.6f;   	// Short term smoothing
static float aslLongAlpha = 0.9f;   	// Long term smoothing
static float altHoldAccSpeedAlpha = 0.5f;
static float altHoldSpeedAlpha = 0.7f;
static float altHoldSpeedDeadband = 3.f;
static float altHoldAccSpeedDeadband = 3.f;
static float altHoldSpeed = 0.f;
static float altHoldAccSpeed = 0.f;
static float altHoldAltSpeed = 0.f;
static float altHoldAccSpeedGain = 0.3f;
static float altHoldAltSpeedGain = 3.f;
static float factorForPowerLevelAndAlt = 10.f;
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
static void updateSpeedByAcceleration();
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

	case ALTHOLD_MODULE_VL53L0X:

		if (!vl53l0xInit()) {
			_DEBUG(DEBUG_NORMAL, "vl53l0x Init failed\n");
			return false;
		}

		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,10.f,1.f,5.f, 0.f);
		setMaxAlt(100);  		//cm

		break;

	case ALTHOLD_MODULE_SRF02:

		if(!srf02Init()){
			_DEBUG(DEBUG_NORMAL, "SRF02 Init failed\n");
			return 	false;
		}
		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,10.f,1.f,5.f, 0.f);
		setMaxAlt(400);		//cm
		
		break;


	case ALTHOLD_MODULE_MS5611:
		
		if(!ms5611Init()){
			_DEBUG(DEBUG_NORMAL, "MS5611 Init failed\n");
			return 	false;
		}
		
		initkalmanFilterOneDimEntity(&altholdKalmanFilterEntry,"ALT", 0.f,30.f,1.f,200.f, 0.f);
		setMaxAlt(500); 	//cm

		break;
		
	default:

		return false;

		break;

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
 * updates speed by acceleration
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void updateSpeedByAcceleration() {

	if (getAltHoldIsReady()) {

		altHoldAccSpeed = altHoldAccSpeed * altHoldAccSpeedAlpha
				+ (1.f - altHoldAccSpeedAlpha)
						* deadband(getAccWithoutGravity() * 100.f,
								altHoldAccSpeedDeadband) * altHoldAccSpeedGain;
		_DEBUG_HOVER(DEBUG_HOVER_ACC_SPEED, "(%s-%d) altHoldAccSpeed=%.3f\n",
				__func__, __LINE__, altHoldAccSpeed);
	}

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
	return asl;
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

	unsigned short data = 0;
	bool result = false;

#if ALTHOLD_CHECK_CYCLE_TIME
	struct timeval tv;
	struct timeval tv2;
#endif

	while (!getLeaveFlyControlerFlag()) {

#if ALTHOLD_CHECK_CYCLE_TIME /*debug: check cycle time of this loop*/
		gettimeofday(&tv,NULL);
		_DEBUG(DEBUG_NORMAL,"duration=%ld us\n",(tv.tv_sec-tv2.tv_sec)*1000000+(tv.tv_usec-tv2.tv_usec));
		tv2.tv_usec=tv.tv_usec;
		tv2.tv_sec=tv.tv_sec;
#endif		

		if (getAltHoldIsReady()) {

			switch (MODULE_TYPE) {

			case ALTHOLD_MODULE_VL53L0X:

				result = vl53l0xGetMeasurementData(&data);
				break;

			case ALTHOLD_MODULE_SRF02:
				
				result = srf02GetMeasurementData(&data);
				break;
				
			case ALTHOLD_MODULE_MS5611:
				
				result = ms5611GetMeasurementData(&data);
				break;

			default:
				
				result = false;
				break;
			}

			if (result) {
				
				data=(unsigned short)kalmanFilterOneDimCalc((float)data,&altholdKalmanFilterEntry);
				aslRaw = (float) data;
				
				updateSpeedByAcceleration();

				asl = asl * aslAlpha + aslRaw * (1.f - aslAlpha);
				aslLong = aslLong * aslLongAlpha
						+ aslRaw * (1.f - aslLongAlpha);
				altHoldAltSpeed = deadband((asl - aslLong),
						altHoldSpeedDeadband) * altHoldAltSpeedGain;
				altHoldSpeed = altHoldAltSpeed * altHoldSpeedAlpha
						+ altHoldAccSpeed * (1.f - altHoldSpeedAlpha);

				pthread_mutex_lock(&altHoldIsUpdateMutex);
				altholdIsUpdate = true;
				pthread_mutex_unlock(&altHoldIsUpdateMutex);

				_DEBUG_HOVER(DEBUG_HOVER_ALT_SPEED,
						"(%s-%d) altHoldAltSpeed=%.3f\n", __func__, __LINE__,
						altHoldAltSpeed);
				_DEBUG_HOVER(DEBUG_HOVER_SPEED, "(%s-%d) altHoldSpeed=%.3f\n",
						__func__, __LINE__, altHoldSpeed);
				_DEBUG_HOVER(DEBUG_HOVER_RAW_ALTITUDE, "(%s-%d) aslRaw=%.3f\n",
						__func__, __LINE__, aslRaw);
				_DEBUG_HOVER(DEBUG_HOVER_FILTERED_ALTITUDE,
						"(%s-%d) asl=%.3f aslLong=%.3f\n", __func__, __LINE__,
						asl, aslLong);

			} else {
				usleep(500);
			}
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

	targetAlt = LIMIT_MIN_MAX_VALUE(LIMIT_MIN_MAX_VALUE(v,0,100)*5, 0,
			getMaxAlt());
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

	ret =
			LIMIT_MIN_MAX_VALUE(
					MIN_POWER_LEVEL+(unsigned short)((float)targetAlt*factorForPowerLevelAndAlt),
					MIN_POWER_LEVEL, MAX_POWER_LEVEL);

	return ret;
}

