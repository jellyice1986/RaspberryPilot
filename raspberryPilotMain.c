/******************************************************************************
The raspberryPilotMain.c in RaspberryPilot project is placed under the MIT license

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
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include "commonLib.h"
#include "motorControl.h"
#include "systemControl.h"
#include "pid.h"
#include "radioControl.h"
#include "flyControler.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "pca9685.h"
#include "altHold.h"
#include "securityMechanism.h"
#include "ahrs.h"

#define MAIN_DELAY_TIMER 200  
#define CONTROL_CYCLE_TIME 2000 //us
#define CHECK_CYCLE_TIME_1 0
#define CHECK_CYCLE_TIME_2 0
#define CHECK_CYCLE_TIME_3 0


bool raspberryPilotInit();

/**
 * RaspberryPilot man function
 *
 * @param
 *		void
 *
 * @return
 *		int
 *
 */
int main() {

#if CHECK_CYCLE_TIME_1 |CHECK_CYCLE_TIME_2
	struct timeval tv1_c;
	struct timeval tv1_l;
	struct timeval tv2_c;
	struct timeval tv2_l;
#endif
	struct timeval tv_c;
	struct timeval tv_l;
	float yrpAttitude[3];
	float pryRate[3];
	float xyzAcc[3];
	float xyzGravity[3];
	float xyzMagnet[3];
	int mpuResult = 0;

	if (!raspberryPilotInit()) {
		return false;
	}

	while (!getLeaveFlyControlerFlag()) {

		gettimeofday(&tv_c,NULL);

#if CHECK_CYCLE_TIME_1 /*debug: check cycle time of this loop*/
		gettimeofday(&tv1_c,NULL);
		_DEBUG(DEBUG_NORMAL,"cycle duration=%ld us\n",(tv1_c.tv_sec-tv1_l.tv_sec)*1000000+(tv1_c.tv_usec-tv1_l.tv_usec));
		tv1_l.tv_usec=tv1_c.tv_usec;
		tv1_l.tv_sec=tv1_c.tv_sec;
#endif

		mpuResult = getYawPitchRollInfo(yrpAttitude, pryRate, xyzAcc,
				xyzGravity, xyzMagnet);

		if (0 == mpuResult
#ifdef MPU_DMP_YAW
				|| 1 == mpuResult || 2 == mpuResult
#endif
				) {

#if CHECK_CYCLE_TIME_2 /*check cycle time of dmp*/
			if(!mpuResult) {
				gettimeofday(&tv2_c,NULL);
				_DEBUG(DEBUG_NORMAL,"dmp duration=%ld us\n",(tv2_c.tv_sec-tv2_l.tv_sec)*1000000+(tv2_c.tv_usec-tv2_l.tv_usec));
				tv2_l.tv_usec=tv2_c.tv_usec;
				tv2_l.tv_sec=tv2_c.tv_sec;
			}
#endif

#ifdef MPU_DMP_YAW
		if(0 == mpuResult)
#endif
			setYaw(yrpAttitude[0]);
			setRoll(yrpAttitude[1]);
			setPitch(yrpAttitude[2]);
			setYawGyro(-pryRate[2]);
			setPitchGyro(pryRate[0]);
			setRollGyro(-pryRate[1]);
			setXAcc(xyzAcc[0]);
			setYAcc(xyzAcc[1]);
			setZAcc(xyzAcc[2]);
			setXGravity(xyzGravity[0]);
			setYGravity(xyzGravity[1]);
			setZGravity(xyzGravity[2]);
			
			_DEBUG(DEBUG_ATTITUDE,
					"(%s-%d) ATT: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n", __func__,
					__LINE__, getRoll(), getPitch(), getYaw());
			_DEBUG(DEBUG_GYRO,
					"(%s-%d) GYRO: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n",
					__func__, __LINE__, getRollGyro(), getPitchGyro(),
					getYawGyro());
			_DEBUG(DEBUG_ACC, "(%s-%d) ACC: x=%3.3f y=%3.3f z=%3.3f\n",
					__func__, __LINE__, getXAcc(), getYAcc(), getZAcc());

    		if((unsigned long)((tv_c.tv_sec-tv_l.tv_sec)*1000000+(tv_c.tv_usec-tv_l.tv_usec)) >= (unsigned long)(getAdjustPeriod()*CONTROL_CYCLE_TIME)){

#if CHECK_CYCLE_TIME_3
				_DEBUG(DEBUG_NORMAL,"duration=%ld us\n",(tv_c.tv_sec-tv_l.tv_sec)*1000000+(tv_c.tv_usec-tv_l.tv_usec));
#endif
				pthread_mutex_lock(&controlMotorMutex);

				if (flySystemIsEnable()) {

					if (getPacketCounter() < MAX_COUNTER) {
						if (getPidSp(&yawAttitudePidSettings) != 321.0) {
							
							if(getFlippingFlag()!= FLIP_NONE){
								motorControlerFlipping();
							}else{
								motorControler();
							}
							
						} else {
							setFlippingFlag(FLIP_NONE);
							setFlippingStep(0);
							setThrottlePowerLevel(getMinPowerLevel());
							setupAllMotorPoewrLevel(getMinPowerLevel(),
									getMinPowerLevel(), getMinPowerLevel(),
									getMinPowerLevel());
						}
					} else {
						setFlippingFlag(FLIP_NONE);
						setFlippingStep(0);
						//security mechanism is triggered while connection is broken
						triggerSecurityMechanism();
					}

				} else {
					setFlippingFlag(FLIP_NONE);
					setFlippingStep(0);
					setThrottlePowerLevel(0);
					setupAllMotorPoewrLevel(0, 0, 0, 0);
				}
				
				pthread_mutex_unlock(&controlMotorMutex);

				tv_l.tv_usec=tv_c.tv_usec;
				tv_l.tv_sec=tv_c.tv_sec;
			}

		}

		usleep(MAIN_DELAY_TIMER);
		
	}

	return 0;
}

/**
 * Init RaspberryPilot
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
bool raspberryPilotInit() {

	if (!piSystemInit()) {
		_ERROR("(%s-%d) Init Raspberry Pi failed!\n", __func__, __LINE__);
		return false;
	}else{
		securityMechanismInit();
		pidInit();
		ahrsInit();
	}

	if (!flyControlerInit()) {
		_ERROR("(%s-%d) Init flyControlerInit failed!\n", __func__, __LINE__);
		return false;
	}

	if (!pca9685Init()) {
		_ERROR("(%s-%d) Init PCA9685 failed!\n", __func__, __LINE__);
		return false;
	}

	if (!mpu6050Init()) {
		_ERROR("(%s-%d) Init MPU6050 failed!\n", __func__, __LINE__);
		return false;
	}

	if (!initAltHold()) {
		_ERROR("(%s-%d) Init altHold failed!\n", __func__, __LINE__);
	}

	if (!radioControlInit()) {
		_ERROR("(%s-%d) radioControler init failed\n", __func__, __LINE__);
		return false;
	}

	_DEBUG(DEBUG_NORMAL, "(%s-%d) Raspberry Pilot init done\n", __func__,
			__LINE__);
	return true;

}

