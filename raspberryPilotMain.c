
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>


#include "motorControl.h"
#include "systemControl.h"
#include "smoother.h"
#include "pid.h"
#include "radioControl.h"
#include "flyControler.h"
#include "verticalHeightHold.h"
#include "mpu6050.h"
#include "safeMachenism.h"

#ifndef MPU_DMP	
	#include "ahrs.h"
#endif



#ifdef MPU_DMP
#define ADJUST_TIMER 2000 // 2 usec *+ DMP period
#else
#define ADJUST_TIMER 500  //5 usec
#endif

int main() {

	struct timeval tv;
	long last_us = 0;
	long last_s = 0;
	short count = 0;
	float yrpAttitude[3];
	float pryRate[3];
	float xyzAcc[3];
	float xyzGravity[3];
	float xyzMagnet[3];
	
	printf("0x11 -> %d\n",checkI2cDeviceIsExist(0x11));
	printf("0x77 -> %d\n",checkI2cDeviceIsExist(0x77));
	printf("0x68 -> %d\n",checkI2cDeviceIsExist(0x68));
	
	setSystemSignalEvent();
	setLeaveFlyControlerFlag(false);
	flyControlerInit();
	initPacketAccCounter();

	if (false==piSystemInit() ) {
			return -1;
	}
	
	if (false == mpu6050Init()) {
		printf("Init DMP failed!\n");
		return -1;
	}
	
#if 0
	if (false==initVerticalHeightHold() ) {
				return -1;
	}

#endif	

	if (pthread_mutex_init(&controlMotorMutex, NULL) != 0) {
		printf("controlMotorMutex init failed\n");
		return -1;
	}

	if(false==radioControlInit()){
		printf("radioControler init failed\n");
		return -1;
	}
	
	pidInit();

#ifndef MPU_DMP	
	ahrsInit();
#endif

	while (true!=getLeaveFlyControlerFlag()) {

#if 0 /*check cycle time of while*/
		gettimeofday(&tv,NULL);
		printf("duration=%d us\n",(tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us));
		last_us=tv.tv_usec;
		last_s=tv.tv_sec;
#endif

		if (0 == getYawPitchRollInfo(yrpAttitude, pryRate, xyzAcc, xyzGravity,xyzMagnet)) {
			
#if 0 /*check cycle time of dmp*/
			gettimeofday(&tv,NULL);
			printf("duration=%d us\n",(tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us));	
			last_us=tv.tv_usec;
			last_s=tv.tv_sec;
#endif
			count++;
			
			recordVerticalSpeed(xyzAcc,xyzGravity);
			//printf("Vertical Speed=%4.3f\n",getVerticalSpeed());

			setZAxisDegree(xyzGravity[2]);
			//printf("Z Axis Degree=%3.3f\n",getZAxisDegree());

#if 0 /*Attitude smoother*/
			addDataToSmoother(roll_attitude_smoother_buffer, yrpAttitude[1]);
			addDataToSmoother(pitch_attitude_smoother_buffer, yrpAttitude[2]);
			addDataToSmoother(yaw_attitude_smoother_buffer, yrpAttitude[0]);
			yrpAttitude[1] = getDataFromSmoother(roll_attitude_smoother_buffer);
			yrpAttitude[2] = getDataFromSmoother(
					pitch_attitude_smoother_buffer);
			yrpAttitude[0] = getDataFromSmoother(yaw_attitude_smoother_buffer);

#endif
			setYaw(yrpAttitude[0]);
			setPitch(yrpAttitude[2]);
			setRoll(yrpAttitude[1]);

 			//printf("Roll_Attitude=%3.3f, Pitch_Attitude=%3.3f, Yaw_Attitude=%3.3f\n",getRoll(),getPitch(),getYaw());

			//float Mag_Yaw = 57.295780 * atan2(xyzMagnet[0]*cos(-getRoll()*(float) M_PI/180.0 ) + xyzMagnet[1]*sin(-getRoll()*(float) M_PI/180.0)*sin(getPitch()*(float) M_PI/180.0) + xyzMagnet[2]*sin(-getRoll()*(float) M_PI/180.0)*cos(getPitch()*(float) M_PI/180.0),xyzMagnet[1]*cos(getPitch()*(float) M_PI/180.0) - xyzMagnet[2]*sin(getPitch()*(float) M_PI/180.0));
			//printf("Mag_Yaw=%.3f\n",Mag_Yaw);

 			
#if 0 /*Gyro smoother*/
			addDataToSmoother(roll_gyro_smoother_buffer, pryRate[1]);
			addDataToSmoother(pitch_gyro_smoother_buffer, pryRate[0]);
			addDataToSmoother(yaw_gyro_smoother_buffer, pryRate[2]);
			pryRate[1] = getDataFromSmoother(roll_gyro_smoother_buffer);
			pryRate[0] = getDataFromSmoother(pitch_gyro_smoother_buffer);
			pryRate[2] = getDataFromSmoother(yaw_gyro_smoother_buffer);
#endif
			setYawGyro(-pryRate[2]);
			setPitchGyro(pryRate[0]);
			setRollGyro(-pryRate[1]);
			//printf("buf: Roll_Gyro=%3.3f, Pitch_Gyro=%3.3f, Yaw_Gyro=%3.3f\n", getRollGyro(),getPitchGyro(),getYawGyro());
			//printf("acc_x=%.3f, acc_y=%.3f acc_z=%.3f total=%.3f\n",xyzAcc[0],xyzAcc[1],xyzAcc[2],fabs(xyzAcc[0])+fabs(xyzAcc[1])+fabs(xyzAcc[2]));
						
			if (count >= getAdjustPeriod()) {
				if(true==flySystemIsEnable()){
					pthread_mutex_lock(&controlMotorMutex);
					
					if(getPacketAccCounter()!=MAX_COUNTER){
						if  (getPidSp(&yawAttitudePidSettings) != 321.0/*getThrottlePowerLevel() > getMinPowerLevel()*/) {
							adjustMotor();
						}else{
							setupAllMotorPoewrLevel(getMinPowerLevel(),
								getMinPowerLevel(), getMinPowerLevel(),
								getMinPowerLevel());
							setThrottlePowerLevel(getMinPowerLevel());
						}
					}else{
						//safe machenism while connection is broken
						int throttleValue=max(getMinPowerLevel(),getThrottlePowerLevel()-5);
						setPidSp(&rollAttitudePidSettings,0.f);
						setPidSp(&pitchAttitudePidSettings,0.f);
						
						setupAllMotorPoewrLevel(throttleValue,
							throttleValue, throttleValue,
							throttleValue);
						setThrottlePowerLevel(throttleValue);
					}
					/*
					printf("getPacketAccCounter=%d rollsp=%3.3f, pitch=%3.3f, getThrottlePowerLevel=%d\n",
						getPacketAccCounter(),
						getPidSp(&rollAttitudePidSettings),
						getPidSp(&pitchAttitudePidSettings),
						getThrottlePowerLevel());
					*/
					pthread_mutex_unlock(&controlMotorMutex);
				}else{
					usleep(1500);
				}
				
				count = 0;
			}

		}

		usleep(ADJUST_TIMER);
	}

	return 0;
}


