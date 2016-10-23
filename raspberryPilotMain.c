
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "commonLib.h"
#include "motorControl.h"
#include "systemControl.h"
#include "pid.h"
#include "radioControl.h"
#include "flyControler.h"
#include "verticalHeightHold.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "pca9685.h"
#include "securityMechanism.h"
#ifndef MPU_DMP	
#include "ahrs.h"
#endif

#ifdef MPU_DMP
#define MAIN_DELAY_TIMER 2000 
#else
#define MAIN_DELAY_TIMER 500  
#endif

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

	struct timeval tv;
	struct timeval tv2;
	struct timeval tv3;
	short count = 0;
	float yrpAttitude[3];
	float pryRate[3];
	float xyzAcc[3];
	float xyzGravity[3];
	float xyzMagnet[3];
	int mpuResult=0;

	if(!raspberryPilotInit()){
		return false;
	}
	
	while (!getLeaveFlyControlerFlag()) {

#if 0 /*debug: check cycle time of this loop*/
		gettimeofday(&tv,NULL);
		printf("duration=%d us\n",(tv.tv_sec-tv2.tv_sec)*1000000+(tv.tv_usec-tv2.tv_usec));
		tv2.tv_usec=tv.tv_usec;
		tv2.tv_sec=tv.tv_sec;
#endif
		mpuResult= getYawPitchRollInfo(yrpAttitude, pryRate, xyzAcc, xyzGravity,xyzMagnet);

		if (0 == mpuResult
#ifdef MPU_DMP_YAW
			|| 1 == mpuResult || 2 == mpuResult
#endif
		) {
			
#if 0 /*check cycle time of dmp*/
			gettimeofday(&tv,NULL);
			printf("duration=%d us\n",(tv.tv_sec-tv3.tv_sec)*1000000+(tv.tv_usec-tv3.tv_usec));	
			tv3.tv_usec=tv.tv_usec;
			tv3.tv_sec=tv.tv_sec;

#endif
			count++;

#ifdef	FEATURE_VH
			recordVerticalSpeed(xyzAcc,xyzGravity);
			//printf("Vertical Speed=%4.3f\n",getVerticalSpeed());
#else
			setZAxisSlope(xyzGravity[2]);
			//printf("Z Axis Degree=%3.3f\n",getZAxisSlope());
#endif

#ifdef MPU_DMP_YAW
			if(0 == mpuResult)
#endif
			setYaw(yrpAttitude[0]);
			setPitch(yrpAttitude[2]);
			setRoll(yrpAttitude[1]);
			setYawGyro(-pryRate[2]);
			setPitchGyro(pryRate[0]);
			setRollGyro(-pryRate[1]);
			
			_DEBUG(DEBUG_ATTI,"(%s-%d) ATT: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n",__func__,__LINE__,getRoll(),getPitch(),getYaw());
			_DEBUG(DEBUG_GYRO,"(%s-%d) GYRO: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n",__func__,__LINE__, getRollGyro(),getPitchGyro(),getYawGyro());
			_DEBUG(DEBUG_ACC,"(%s-%d) ACC: x=%3.3f y=%3.3f z=%3.3f\n",__func__,__LINE__,xyzAcc[0],xyzAcc[1],xyzAcc[2]);
						
			if (count >= getAdjustPeriod()) {
				if(true==flySystemIsEnable()){
					
					pthread_mutex_lock(&controlMotorMutex);
					if(getPacketCounter()!=MAX_COUNTER){
						if  (getPidSp(&yawAttitudePidSettings) != 321.0) {
							adjustMotor();
						}else{
							setupAllMotorPoewrLevel(getMinPowerLevel(),
								getMinPowerLevel(), getMinPowerLevel(),
								getMinPowerLevel());
							setThrottlePowerLevel(getMinPowerLevel());
						}
					}else{
						//security mechanism is triggered while connection is broken
						triggerSecurityMechanism();
					}
					pthread_mutex_unlock(&controlMotorMutex);
				}
				
				count = 0;
				
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
bool raspberryPilotInit(){

	flyControlerInit();
	securityMechanismInit();

	if (!piSystemInit()) {
		_ERROR("(%s-%d) Init Raspberry Pi failed!\n",__func__,__LINE__);
		return false;
	}
		
	if (!mpu6050Init()) {
		_ERROR("(%s-%d) Init MPU6050 failed!\n",__func__,__LINE__);
		return false;
	}
	if(!pca9685Init()){
		_ERROR("(%s-%d) Init PCA9685 failed!\n",__func__,__LINE__);
		return false;
	}
		
#ifdef FEATURE_VH
	if(checkI2cDeviceIsExist(MS5611_ADDR)){
		_DEBUG(DEBUG_NORMAL,"MS5611 exist\n",__func__,__LINE__);
	}else{
		_ERROR("(%s-%d) MS5611 dowsn't exist\n",__func__,__LINE__);
		//return false;
	}
	
	if (!initVerticalHeightHold() ) {
		return false;
	}
#endif	
	
	if (pthread_mutex_init(&controlMotorMutex, NULL) != 0) {
		_ERROR("(%s-%d) controlMotorMutex init failed\n",__func__,__LINE__);
		return false;
	}
	
	if(!radioControlInit()){
		_ERROR("(%s-%d) radioControler init failed\n",__func__,__LINE__);
		return false;
	}
		
	pidInit();
	
#ifndef MPU_DMP	
	ahrsInit();
#endif

	_DEBUG(DEBUG_NORMAL,"Raspberry Pilot init down\n",__func__,__LINE__);
	return true;

}

