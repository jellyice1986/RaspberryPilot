#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "motorControl.h"
#include "pid.h"
#include "systemControl.h"
#include "verticalHeightHold.h"
#include "mpu6050.h"
#include "flyControler.h"



#define DEFAULT_ADJUST_PERIOD 1
#define DEFAULT_GYRO_LIMIT 20
#define DEFAULT_ANGULAR_LIMIT 500



static float getThrottleOffset();
static void getAttitudePidOutput();
static void convertThrottleByVHH();
static float getThrottleOffsetByVHH();

pthread_mutex_t controlMotorMutex;


static bool leaveFlyControler;
static float rollAttitudeOutput;
static float pitchAttitudeOutput;
static float yawAttitudeOutput;
static float verticalHeightOutput;
static unsigned short adjustPeriod;
static float angularLimit;
static float gyroLimit;
static float yawCenterPoint;
static float cosz;


/**
* Init paramtes and states of flyControler 
*/
void flyControlerInit(){
	setAdjustPeriod(DEFAULT_ADJUST_PERIOD);
	setGyroLimit(DEFAULT_GYRO_LIMIT);
	setAngularLimit(DEFAULT_ANGULAR_LIMIT);
	setMotorGain(SOFT_PWM_CCW1, 1);
	setMotorGain(SOFT_PWM_CW1, 1);
	setMotorGain(SOFT_PWM_CCW2, 1);
	setMotorGain(SOFT_PWM_CW2, 1);
	rollAttitudeOutput = 0;
	pitchAttitudeOutput = 0;
	yawAttitudeOutput = 0;
	verticalHeightOutput=0;
	setZAxisDegree(0.0);
}

void setLeaveFlyControlerFlag(bool v){
	leaveFlyControler=v;
}

bool getLeaveFlyControlerFlag(){
	return leaveFlyControler;
}


/**
 *  get PID output of attitude, this output will be used as a angular velocity
 *
 */
void getAttitudePidOutput() {

	rollAttitudeOutput = pidCalculation(&rollAttitudePidSettings, getRoll(),true);
	rollAttitudeOutput = LIMIT_MIN_MAX_VALUE(rollAttitudeOutput,
			-getGyroLimit(), getGyroLimit());
	pitchAttitudeOutput = pidCalculation(&pitchAttitudePidSettings, getPitch(),true);
	pitchAttitudeOutput = LIMIT_MIN_MAX_VALUE(pitchAttitudeOutput,
			-getGyroLimit(), getGyroLimit());
	yawAttitudeOutput = pidCalculation(&yawAttitudePidSettings, yawTransform(getYaw()),true);
	yawAttitudeOutput = LIMIT_MIN_MAX_VALUE(yawAttitudeOutput,
			-getGyroLimit(), getGyroLimit());
	//printf("shift: roll=%3.3f, pitch=%3.3f, yaw=%3.3f\n",rollAttitudePidSettings.spShift,pitchAttitudePidSettings.spShift,yawAttitudePidSettings.spShift);
//	printf("rollAttitudeOutput=%3.5f, pitchAttitudeOutput=%3.5f, yawAttitudeOutput=%3.5f\n",*rollAttitudeOutput,*pitchAttitudeOutput,*yawAttitudeOutput);

}

/**
 *  get PID output of angular velocity by using current attitude
 *  @param rollAttitudeOutput
 *  		PID output of roll attitude
 *  @param pitchAttitudeOutput
 *  		PID output of pitch attitude
 * 	@param yawAttitudeOutput
 * 			PID output of yaw attitude
 * 	@param rollRateOutput
 * 			point to record PID output of roll angular velocity
 * 	@param pitchRateOutput
 * 			point to record PID output of pitch angular velocity
 * 	@param yawRateOutput
 * 			point to record PID output of yaw angular velocity
 */
void getRatePidOutput(float rollAttitudeOutput, float pitchAttitudeOutput,
		float yawAttitudeOutput, float *rollRateOutput, float *pitchRateOutput,
		float *yawRateOutput) {

	setPidSp(&rollRatePidSettings, rollAttitudeOutput);
	setPidSp(&pitchRatePidSettings, pitchAttitudeOutput);
	setPidSp(&yawRatePidSettings, yawAttitudeOutput);
	*rollRateOutput = pidCalculation(&rollRatePidSettings, getRollGyro(),true);
	*pitchRateOutput = pidCalculation(&pitchRatePidSettings, getPitchGyro(),true);
	*yawRateOutput = pidCalculation(&yawRatePidSettings, getYawGyro(),true);
//	printf("rollRateOutput=%3.5f, pitchRateOutput=%3.5f, yawRateOutput=%3.5f\n\n",*rollRateOutput,*pitchRateOutput,*yawRateOutput);
}


void getVerticalHeightPidOutput () {
	verticalHeightOutput = pidCalculation(&verticalHeightSettings, getVerticalHeight(),true);
	//verticalHeightOutput = LIMIT_MIN_MAX_VALUE(verticalHeightOutput,-500, 500);
  // TODO
}

void getVerticalSpeedPidOutput(float verticalHeightOutput,	float *verticalSpeedOutput) {
	//TODO	
	setPidSp(&verticalSpeedSettings, verticalHeightOutput);
	*verticalSpeedOutput = pidCalculation(&verticalSpeedSettings, getVerticalSpeed(),true);
	//printf("getVerticalSpeed=%f\n",getVerticalSpeed());//In cm
}

static float altHoldErrMax          = 300.0;   // max cap on current estimated altitude vs target altitude in cm
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vSpeedAccFac           = -48.f;  // multiplier
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float pidAslFac              = 1; // relates meters asl to thrust
static float altHoldPIDVal=0.f;
static bool initVHH=0.f;
static float altHoldErr=0.f;  
static float altHoldTarget          = 0.f;    // Target altitude
static unsigned short altHoldMinThrust    = 2014; // minimum hover thrust - not used yet
static unsigned short altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static unsigned short altHoldMaxThrust    = 4014; // max altitude hold thrust



void setInitVHH(bool v){
	initVHH=v;
}

bool getInitVHH(void){
	return initVHH;
}

static float alphaVHH=0.4;
static float safeRange=0.7;

float getThrottleOffsetByVHH(){

		float pidOutput=0.f;
		
		setPidSp(&verticalSpeedSettings, 0);
		pidOutput = pidCalculation(&verticalSpeedSettings, getVerticalSpeed(),true);
		LIMIT_MIN_MAX_VALUE(pidOutput,-200,200);
		//printf("pidOutput=%f\n",pidOutput);
		return pidOutput;
}

void convertThrottleByVHH(){

	float heightHoldErr=0.f;
	float pidOutput=0.f;
	float throttle=0.f;

	if(true==getInitVHH()){
		throttle=0.f;
	}
	if(true==getVerticalHeightHoldEnable()){
		heightHoldErr = constrain(deadband(getPidSp(&verticalHeightSettings)-getVerticalHeight(), errDeadband),
								  -altHoldErrMax, altHoldErrMax);
		setPidError(&verticalHeightSettings, heightHoldErr);

		verticalHeightOutput = pidCalculation(&verticalHeightSettings, getVerticalHeight(),false);

		getVerticalSpeedPidOutput(verticalHeightOutput,	&pidOutput);

		//throttle= alphaVHH*getThrottlePowerLevel()+(1-alphaVHH)*(getThrottlePowerLevel()+pidOutput);
		throttle=getThrottlePowerLevel()+pidOutput;
		throttle= max(getMinPowerLevel(), min(getMaxPowerLeve()-500,throttle));
		//printf("throttle=%f\n",throttle);
		setThrottlePowerLevel((unsigned short)throttle);
}
}

void convertThrottleByVHH2(){
	//TODO

	if(true==getInitVHH()){

		setInitVHH(false);
		altHoldPIDVal=0.f;
		 // Reset altHoldPID
    	//altHoldPIDVal = pidCalculation(&verticalHeightSettings, getVerticalHeight(), false));
	}

	if(true==getVerticalHeightHoldEnable()){
			unsigned short actuatorThrust=0;

			// Compute error (current - target), limit the error
			altHoldErr = constrain(deadband(getPidSp(&verticalHeightSettings)-getVerticalHeight(), errDeadband),
								  -altHoldErrMax, altHoldErrMax);
			setPidError(&verticalHeightSettings, altHoldErr);

			altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((getVerticalSpeed() * vSpeedAccFac) +
				pidCalculation(&verticalHeightSettings, getVerticalHeight(), false));

			actuatorThrust =  max(getMinPowerLevel(), min(getMaxPowerLeve(),getMinPowerLevel()+(unsigned short)(altHoldPIDVal*pidAslFac)));
			setThrottlePowerLevel(actuatorThrust);
			//printf("altHoldErr=%f altHoldPIDVal=%f (getVerticalSpeed() * vSpeedAccFac)=%f getThrottlePowerLevel()=%d,getVerticalHeight()=%f\n",altHoldErr,altHoldPIDVal,(getVerticalSpeed() * vSpeedAccFac),getThrottlePowerLevel(),getVerticalHeight());
	}
	
}


/**
 *  adjust motor power to stabilize quadcopter
 */
void adjustMotor() {

	float rollRateOutput = 0.f;
	float rollCcw1 = 0.f;
	float rollCcw2 = 0.f;
	float rollCw1 = 0.f;
	float rollCw2 = 0.f;

	float pitchRateOutput = 0.f;
	float pitchCcw1 = 0.f;
	float pitchCcw2 = 0.f;
	float pitchCw1 = 0.f;
	float pitchCw2 = 0.f;

	float yawRateOutput = 0.f;
	float yawCcw1 = 0.f;
	float yawCcw2 = 0.f;
	float yawCw1 = 0.f;
	float yawCw2 = 0.f;

	float outCcw1 = 0.f;
	float outCcw2 = 0.f;
	float outCw1 = 0.f;
	float outCw2 = 0.f;

	float maxLimit = 0.f;
	float minLimit = 0.f;
	float throttleOffset = 0.f;
#if 0
	if(true==getVerticalHeightHoldEnable()){
		throttleOffset=getThrottleOffsetByVHH();
	}
#endif
	//throttleOffset=getThrottleOffset();
	
	maxLimit = (float) min(
			(getThrottlePowerLevel() + throttleOffset) + getAdjustPowerLeveRange(),
			getMaxPowerLeve());
	minLimit = (float) max(
			(getThrottlePowerLevel() + throttleOffset) - getAdjustPowerLeveRange(),
			getMinPowerLevel());

	getAttitudePidOutput();
	getRatePidOutput(rollAttitudeOutput, pitchAttitudeOutput, yawAttitudeOutput,
			&rollRateOutput, &pitchRateOutput, &yawRateOutput);

	// rollCa>0
	//    -  CCW2   CW2   +
	//            X
	//    -   CW1   CCW1  +
	//            H
	//
	// rollCa<0
	//    +  CCW2   CW2    -
	//            X
	//    +   CW1   CCW1  -
	//            H

	rollCcw1 = rollRateOutput;
	rollCcw2 = -rollRateOutput;
	rollCw1 = -rollRateOutput;
	rollCw2 = rollRateOutput;

	// pitchCa>0
	//    +  CCW2   CW2    +
	//            X
	//    -   CW1   CCW1   -
	//            H
	//
	//pitchCa<0
	//    -  CCW2   CW2   -
	//            X
	//    +   CW1   CCW1  +
	//            H

	pitchCcw1 = -pitchRateOutput;
	pitchCcw2 = pitchRateOutput;
	pitchCw1 = -pitchRateOutput;
	pitchCw2 = pitchRateOutput;

	if (getPidSp(&yawAttitudePidSettings) != 321.0) {

		// yawCa>0
		//    +   CCW2   CW2    -
		//             X
		//    -    CW1   CCW1   +
		//             H
		//
		// yawCa<0
		//    -  CCW2    CW2  +
		//            X
		//    +   CW1   CCW1  -
		//            H

		yawCcw1 = yawRateOutput;
		yawCcw2 = yawRateOutput;
		yawCw1 = -yawRateOutput;
		yawCw2 = -yawRateOutput;
	}

	outCcw1 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCcw1+pitchCcw1+yawCcw1,-getAdjustPowerLimit(),getAdjustPowerLimit());
	outCcw2 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCcw2+pitchCcw2+yawCcw2,-getAdjustPowerLimit(),getAdjustPowerLimit());
	outCw1 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCw1+pitchCw1+yawCw1,-getAdjustPowerLimit(),getAdjustPowerLimit());
	outCw2 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCw2+pitchCw2+yawCw2,-getAdjustPowerLimit(),getAdjustPowerLimit());

	outCcw1 =
			getMotorGain(SOFT_PWM_CCW1) * LIMIT_MIN_MAX_VALUE(outCcw1, minLimit, maxLimit);
	outCcw2 =
			getMotorGain(SOFT_PWM_CCW2) * LIMIT_MIN_MAX_VALUE(outCcw2, minLimit, maxLimit);
	outCw1 = getMotorGain(SOFT_PWM_CW1) * LIMIT_MIN_MAX_VALUE(outCw1, minLimit, maxLimit);
	outCw2 = getMotorGain(SOFT_PWM_CW2) * LIMIT_MIN_MAX_VALUE(outCw2, minLimit, maxLimit);

	setupCcw1MotorPoewrLevel((unsigned short) outCcw1);
	setupCcw2MotorPoewrLevel((unsigned short) outCcw2);
	setupCw1MotorPoewrLevel((unsigned short) outCw1);
	setupCw2MotorPoewrLevel((unsigned short) outCw2);

#if 0 /*Debug*/
	//printf("shift: roll=%3.3f, pitch=%3.3f, yaw=%3.3f\n",rollAttitudePidSettings.spShift,pitchAttitudePidSettings.spShift,yawAttitudePidSettings.spShift);
	printf("getThrottlePowerLevel=%d\n ccw1:%3.3f, cw1:%3.3f cw2:%3.3f, ccw2:%3.3f\n\n",
			getThrottlePowerLevel(),
			outCcw1,
			outCw1,
			outCw2,
			outCcw2);
#endif

}


float getZAxisDegree(){
	return cosz;
}

void setZAxisDegree(float degree){
	cosz=degree;
}

/**
 *  get throttle compensation
 */
float getThrottleOffset() {
	float offset = 0;
	if (getZAxisDegree() <= 0.0) {
		//inverted or vertical
		offset = 0.0;
	} else {
		offset = (((1.0 - getZAxisDegree()) >= 0.3) ? 0.3 : 1.0 - getZAxisDegree())
				* ((float) (getThrottlePowerLevel() - getMinPowerLevel()));
	}
	//printf("cosz=%3.3f getThrottlePowerLevel=%d getMinPowerLeveRange=%d offset=%3.3f\n",cosz,getThrottlePowerLevel(),getMinPowerLevel(),offset);
	return offset;
}

/**
* set center point of yaw
* @param point
* 		center point of yaw
*/
void setYawCenterPoint(float point){
	float yawCenterPoint1=point;
	if(yawCenterPoint1>180.0){
		yawCenterPoint1=yawCenterPoint1-360.0;
	}else if(yawCenterPoint1<-180.0){
		yawCenterPoint1=yawCenterPoint1+360.0;
	}
	yawCenterPoint=yawCenterPoint1;
}

/**
* get center point of yaw
* @return
*               center point of yaw
*/
float getYawCenterPoint(){
        return yawCenterPoint;
}


/**
* transform yaw value by using YawCenterPoint
* @param originPoint
*               the yaw value befor transform
*
* @return
*		the yaw value after transform
*/
float yawTransform(float originPoint){
	float output=originPoint-yawCenterPoint;
	if(output>180.0){
		output=output-360.0;
	}else if(output<-180.0){
		output=output+360.0;
	}
	//printf("yawCenterPoint=%3.3f, originPoint=%3.3f, output=%3.3f\n",yawCenterPoint,originPoint,output);
	return output;
}

void setGyroLimit(float v) {
	gyroLimit = v;
}

float getGyroLimit() {
	return gyroLimit;
}

unsigned short getAdjustPeriod() {
	return adjustPeriod;
}

void setAdjustPeriod(unsigned short ms) {
	adjustPeriod = ms;
}

float getAngularLimit() {
	return angularLimit;
}

void setAngularLimit(float angular) {
	angularLimit = angular;
}



