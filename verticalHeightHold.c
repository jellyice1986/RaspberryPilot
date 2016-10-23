
#ifdef FEATURE_VH

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>


#include "commonLib.h"
#include "ms5611.h"
#include "verticalHeightHold.h"

static float aslAlpha               = 0.92;   // Short term smoothing
static float aslAlphaLong           = 0.93;   // Long term smoothing
static float vSpeedASL 				= 0.0;
static float vSpeedAcc 				= 0.0;
static float vSpeed    				= 0.0;   // Vertical speed (world frame) integrated from vertical acceleration
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vAccDeadband           = 0.005; // Vertical acceleration deadband
static float vBiasAlpha             = 0.7; 	 // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float vSpeedLimit            = 20;	 // used to constrain vertical velocity
static float asl=0.f;
static float aslLong=0.f;
static float asl_last=0.f;
static float aslRaw=0.f;
static short maxRisenVerticalHeight; //cm
static short startVerticalHeight;
static bool verticalHeightHoldEnable;
static pthread_t ms5611ThreadId;


static float sensfusion6GetAccZWithoutGravity(float *xyzAcc, float *xyzGravity);


bool initVerticalHeightHold(){

	if (false==ms5611Init() ) {
		return false;
	}

	if (pthread_create(&ms5611ThreadId, NULL, getDataFromMs5611Thread, 0)) {
			printf("ms5611 create failed\n");
			return false;
	} else {
			printf("start ms5611 thread...\n");
	}
	
	setVerticalHeightHoldEnable(false);
	setMaxRisenVerticalHeight(600);//In cm
	setStartRisenVerticalHeight(0);//cm

	return true;
}



float recordVerticalSpeed(float *xyzAcc, float *xyzGravity){
	struct timeval tv;	
	static long last_us = 0;
	static long last_s = 0;
	float a=0.0; 
	
	gettimeofday(&tv,NULL);
	//printf("duration=%d us\n",(tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us));
	//printf("duration=%f s\n",((tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us))*0.000001);
	a=sensfusion6GetAccZWithoutGravity(xyzAcc, xyzGravity);
	vSpeed += deadband(a* 0.02/*((tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us))*0.000001*/, vAccDeadband);;
	//printf("aX=%f, aY=%f, aZ=%f, gvX=%f, gvY=%f, gvZ=%f\n",xyzAcc[0],xyzAcc[1],xyzAcc[2],xyzGravity[0],xyzGravity[1],xyzGravity[2]);
	//printf("vSpeed=%f\n",vSpeed*100.f);
	// Estimate vertical speed based on Acc - fused with baro to reduce drift
	vSpeed = LIMIT_MIN_MAX_VALUE(vSpeed, -vSpeedLimit, vSpeedLimit);
	//printf("vSpeed=%f, ",vSpeed*100);
	vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
	//printf(" vSpeedASL=%f\n",vSpeedASL*100);
	vSpeedAcc = vSpeed;
	//printf("vSpeedAcc=%f\n",vSpeedAcc*100.f);

	last_us=tv.tv_usec;
	last_s=tv.tv_sec;
}

float getVerticalSpeed(){
	return vSpeedAcc*100.f;//In cm
}


float getVerticalHeight(){
	return aslLong*100.f;//In cm
}

void setMaxRisenVerticalHeight(short v){
	maxRisenVerticalHeight=v;
}
short getMaxRisenVerticalHeight(){
	return maxRisenVerticalHeight;
}

void setStartRisenVerticalHeight(short v){
	startVerticalHeight=v;
}
short getStartRisenVerticalHeight(){
	return startVerticalHeight;
}

void setVerticalHeightHoldEnable(bool v){
	verticalHeightHoldEnable=v;
}

bool getVerticalHeightHoldEnable(){
	return verticalHeightHoldEnable;
}

void *getDataFromMs5611Thread(void *arg){

	//struct timeval tv;
	//long last_us = 0;
	//long last_s = 0;

	while(1){
		//gettimeofday(&tv,NULL);
		aslRaw=getAltitude();
		asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
		aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);
		//printf("aslRaw=%5.3f, asl=%5.3f aslLong=%5.3f\n",aslRaw,asl,aslLong);
		
		// Estimate vertical speed based on successive barometer readings. 
        vSpeedASL =deadband((asl-aslLong)/0.2, vSpeedASLDeadband);
		 
		//printf("asl=%f, aslLong=%f, vSpeedASL=%f\n",asl,aslLong,vSpeedASL*100.f);
		//printf("interval=%f\n",(((tv.tv_sec-last_s)*1000000+(tv.tv_usec-last_us))*0.000001));
        //last_us=tv.tv_usec;
        //last_s=tv.tv_sec;
        usleep(10000);
	}
}


float sensfusion6GetAccZWithoutGravity(float *xyzAcc, float *xyzGravity)
{
  return ((xyzAcc[0]*xyzGravity[0] + xyzAcc[1]*xyzGravity[1] + xyzAcc[2]*xyzGravity[2]) - 1.0);
}
#endif

