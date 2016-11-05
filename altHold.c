
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include "commonLib.h"
#include "flyControler.h"
#include "vl53l0x.h"
#include "mpu6050.h"
#include "altHold.h"

#define MODULE_TYPE ALTHOLD_MODULE_VL53L0X

static pthread_t altHoldThreadId;
static float aslRaw					= 0.f;
static float asl					= 0.f;
static float aslLong				= 0.f;
static float aslAlpha               = 0.3f;   // Short term smoothing
static float aslLongAlpha           = 0.6f;   // Long term smoothing
static float altHoldAccSpeedAlpha 	= 0.8f;
static float altHoldSpeedAlpha  	= 0.6f; 
static float altHoldSpeedDeadband   = 0.f;
static float altHoldAccSpeedDeadband	= 3.f;
static float altHoldSpeed 			= 0.f;
static float altHoldAccSpeed 		= 0.f;
static float altHoldAltSpeed 		= 0.f;
static float altHoldAccSpeedGain 		= 0.3f;
static float altHoldAltSpeedGain 		= 3.f;
static bool altHoldIsReady			= false;
static unsigned short maxAlt		= 50; //cm
static unsigned short startAlt   	= 0; //cm
static struct timeval tv_last;

static void setAltHoldIsReady(bool v);
static void setMaxAlt(unsigned short v);
static unsigned short getMaxAlt();
static void setStartAlt(unsigned short v);
static unsigned short getStartAlt();
static void *altHoldThread(void *arg);
static float getAccWithoutGravity();
void updateSpeedByAcceleration();

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
bool initAltHold(){

	switch(MODULE_TYPE){
		case ALTHOLD_MODULE_VL53L0X:
		
			if(!vl53l0xInit()){
				_DEBUG(DEBUG_NORMAL,"vl53l0x Init failed\n");
				setAltHoldIsReady(false);
				return false;
			}
			
			setMaxAlt(60);//In cm
			setStartAlt(0);//cm
			
		break;
		
		case ALTHOLD_MODULE_MS5611:
		
			//unavailable
			setAltHoldIsReady(false);
			return false;
			
		break;
		
		default:
		
			setAltHoldIsReady(false);
			return false;
			
		break;
	}

	if (pthread_create(&altHoldThreadId, NULL, altHoldThread, 0)) {
			_DEBUG(DEBUG_NORMAL,"altHold thread create failed\n");
			return false;
	} else {
			_DEBUG(DEBUG_NORMAL,"start altHold thread...\n");
	}
	
	setAltHoldIsReady(true);

	return true;
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
bool getAltHoldIsReady(){
	return altHoldIsReady;
}

/**
* set the flag to indicate whether althold is ready or not
*
* @param
* 		altHold is ready or not
*
* @return 
*		void
*		
*/
void setAltHoldIsReady(bool v){
	altHoldIsReady=v;
}

/**
* set the maximum value of altitude
*
* @param
* 		maximum value of altitude
*
* @return 
*		void
*		
*/
void setMaxAlt(unsigned short v){
	maxAlt=v;
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
unsigned short getMaxAlt(){
	return maxAlt;
}

/**
* set the start value of altitude
*
* @param
* 		start value of altitude
*
* @return 
*		void
*		
*/
void setStartAlt(unsigned short v){
	startAlt=v;
}

/**
* get the start value of altitude
*
* @param
* 		void
*
* @return 
*		start value of altitude
*		
*/
unsigned short getStartAlt(){
	return startAlt;
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
float getAccWithoutGravity(){
	return (getXAcc()*getXGravity()+getYAcc()*getYGravity()+getZAcc()*getZGravity()-1.f);
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
void updateSpeedByAcceleration(){

	if(getAltHoldIsReady()){

		struct timeval tv;

		gettimeofday(&tv,NULL);

		if(0!=tv_last.tv_sec){
			altHoldAccSpeed=altHoldAccSpeed*altHoldAccSpeedAlpha+(1.f-altHoldAccSpeedAlpha)*deadband(getAccWithoutGravity()*100.f,altHoldAccSpeedDeadband)*altHoldAccSpeedGain;	
			_DEBUG_HOVER(DEBUG_HOVER_ACC_SPEED,"(%s-%d) altHoldAccSpeed=%.3f\n",__func__,__LINE__,altHoldAccSpeed);
		}
		
		tv_last.tv_usec=tv.tv_usec;
		tv_last.tv_sec=tv.tv_sec;
	}
}

/**
* thread of AltHolt, this thread update altitude and vertical speed periodically
*
* @param
* 		arg
*
* @return 
*		pointer
*		
*/
void *altHoldThread(void *arg){

	unsigned short data=0;
	bool result=false;
		
	while(!getLeaveFlyControlerFlag()){
		
		switch(MODULE_TYPE){
			
			case ALTHOLD_MODULE_VL53L0X:
		
				result=vl53l0xGetMeasurementData(&data);
				aslRaw=(float)data*0.1f;
			
			break;
		
			case ALTHOLD_MODULE_MS5611:
			//unavailable
				result=false;
			break;
		
			default:
				result=false;
			break;
		}
		
		if(result){

			updateSpeedByAcceleration();

			asl = asl * aslAlpha + aslRaw * (1.f - aslAlpha);
			aslLong = aslLong * aslLongAlpha + aslRaw * (1.f - aslLongAlpha);
			altHoldAltSpeed=deadband((asl-aslLong), altHoldSpeedDeadband)*altHoldAltSpeedGain;
			altHoldSpeed = altHoldAltSpeed * altHoldSpeedAlpha +  altHoldAccSpeed* (1.f - altHoldSpeedAlpha);

			_DEBUG_HOVER(DEBUG_HOVER_ALT_SPEED,"(%s-%d) altHoldAltSpeed=%.3f\n",__func__,__LINE__,altHoldAltSpeed);
			_DEBUG_HOVER(DEBUG_HOVER_SPEED,"(%s-%d) altHoldSpeed=%.3f\n",__func__,__LINE__,altHoldSpeed);
			_DEBUG_HOVER(DEBUG_HOVER_RAW_ALTITUDE,"(%s-%d) aslRaw=%.3f\n",__func__,__LINE__,aslRaw);
			_DEBUG_HOVER(DEBUG_HOVER_FILTERED_ALTITUDE,"(%s-%d) asl=%.3f aslLong=%.3f\n",__func__,__LINE__,asl,aslLong);
			
		}
		
		usleep(25000);
	}
	
	pthread_exit((void *)1234);
}

