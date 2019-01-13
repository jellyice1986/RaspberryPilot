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
#include "pca9685.h"
#include "altHold.h"
#include "securityMechanism.h"
#include "ahrs.h"
#include "attitudeUpdate.h"

#define CONTROL_CYCLE_TIME 500
#define CHECK_RASPBERRYPILOT_LOOP_TIME 0

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

	struct timeval tv_c;
	struct timeval tv_l;
	unsigned long timeDiff=0;

	if (!raspberryPilotInit()) {
		return false;
	}

	gettimeofday(&tv_l,NULL);

	while (!getLeaveFlyControlerFlag()) {

		gettimeofday(&tv_c,NULL);
		timeDiff=GET_USEC_TIMEDIFF(tv_c,tv_l);

		if(timeDiff >=((unsigned long) (getAdjustPeriod() * CONTROL_CYCLE_TIME))){

#if CHECK_RASPBERRYPILOT_LOOP_TIME
			_DEBUG(DEBUG_NORMAL,"RaspberryPilot main duration=%ld us\n",timeDiff);
#endif
			pthread_mutex_lock(&controlMotorMutex);

			if(!magnetCalibrationIsEnable()){

				attitudeUpdate();

				if (flySystemIsEnable()){

					disenableMagnetCalibration();
					
					if (getPacketCounter() < MAX_COUNTER) {
						
						if (getPidSp(&yawAttitudePidSettings) != 321.0) {
								
								motorControler();
										
						} else {

							setThrottlePowerLevel(getMinPowerLevel());
							setupAllMotorPoewrLevel(getMinPowerLevel(),
									getMinPowerLevel(), getMinPowerLevel(),
									getMinPowerLevel());
						}
					} else {

						//security mechanism is triggered while connection is broken
						triggerSecurityMechanism();
					}

				} else {

					setThrottlePowerLevel(0);
					setupAllMotorPoewrLevel(0, 0, 0, 0);
				}
			}else{

				magnetCalibrationGetImuRawData();
			}

			pthread_mutex_unlock(&controlMotorMutex);
			
			UPDATE_LAST_TIME(tv_c,tv_l);

		}

		usleep(500);
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

	if(!altitudeUpdateInit()){
		_ERROR("(%s-%d) attitude update failed\n", __func__, __LINE__);
		return false;
	}

	_DEBUG(DEBUG_NORMAL, "(%s-%d) Raspberry Pilot init done\n", __func__,
			__LINE__);
	return true;

}

