/******************************************************************************
The flyControler.c in RaspberryPilot project is placed under the MIT license

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
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <sys/time.h>
#include "commonLib.h"
#include "motorControl.h"
#include "systemControl.h"
#include "mpu6050.h"
#include "pid.h"
#include "altHold.h"
#include "flyControler.h"

#define DEFAULT_ADJUST_PERIOD 1
#define DEFAULT_GYRO_LIMIT 50
#define DEFAULT_ANGULAR_LIMIT 5000
#define FLIP_THRESHOLD 	30.f
#define FLIP_DELAY 		5 //sec
#define FLIP_POWER 		300.f

float getSlopeThrottleOffset();
static void getAttitudePidOutput();
void getAltHoldAltPidOutput();
void getAltHoldSpeedPidOutput(float *altHoldSpeedOutput);
float getThrottleOffsetByAltHold(bool updateAltHoldOffset);
void setFlippingIsEnable(bool val);
void setFlipThreadHold(unsigned char v);
void setFlipDelay(unsigned char v);
unsigned char getFlipDelay();
void setFlipPower(unsigned short v);

pthread_mutex_t controlMotorMutex;

static bool leaveFlyControler;
static float rollAttitudeOutput;
static float pitchAttitudeOutput;
static float yawAttitudeOutput;
static float altHoltAltOutput;
static unsigned short adjustPeriod;
static float angularLimit;
static float gyroLimit;
static float yawCenterPoint;
static float maxThrottleOffset;
static float altitudePidOutputLimitation;
static float SlopeThrottleOffsetGain=1.f;
static float altStartPoint;

//flip parameter
static unsigned char flippingFlag;
static unsigned char flipStep;
static bool flipIsEnable;
static unsigned char flipThreshold;
static unsigned char flipDelay;
static unsigned short flipPower;

/**
 * Init paramtes and states for flyControler
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool flyControlerInit() {

	if (pthread_mutex_init(&controlMotorMutex, NULL) != 0) {
		_ERROR("(%s-%d) controlMotorMutex init failed\n", __func__, __LINE__);
		return false;
	}

	setLeaveFlyControlerFlag(false);
	disenableFlySystem();
	setAdjustPeriod(DEFAULT_ADJUST_PERIOD);
	setGyroLimit(DEFAULT_GYRO_LIMIT);
	setAngularLimit(DEFAULT_ANGULAR_LIMIT);
	setMotorGain(SOFT_PWM_CCW1, 1);
	setMotorGain(SOFT_PWM_CW1, 1);
	setMotorGain(SOFT_PWM_CCW2, 1);
	setMotorGain(SOFT_PWM_CW2, 1);
	setAltitudePidOutputLimitation(15.f); // 15 cm/sec
	rollAttitudeOutput = 0.f;
	pitchAttitudeOutput = 0.f;
	yawAttitudeOutput = 0.f;
	altHoltAltOutput = 0.f;
	maxThrottleOffset = 1000.f;

	//init flip parameter
	flippingFlag=0x0;
	flipStep=0;
	setFlippingIsEnable(false);
	setFlipThreadHold(FLIP_THRESHOLD);
	setFlipDelay(FLIP_DELAY);
	setFlipPower(FLIP_POWER);
	
	return true;
}

/**
 * set a value to indicate whether the pilot is halting or not
 *
 * @param v
 * 		value
 *
 * @return
 *		void
 *
 */
void setLeaveFlyControlerFlag(bool v) {
	leaveFlyControler = v;
}

/**
 * get the value to indicate whether the pilot is halting or not
 *
 * @param
 * 		void
 *
 * @return
 *		value
 *
 */
bool getLeaveFlyControlerFlag() {
	return leaveFlyControler;
}

/**
 *  get the output of attitude PID controler, this output will become  a input for angular velocity PID controler
 *
 * @param
 * 		void
 *
 * @return 
 *		value
 *
 */
void getAttitudePidOutput() {

	rollAttitudeOutput = LIMIT_MIN_MAX_VALUE(
			pidCalculation(&rollAttitudePidSettings, getRoll(),true,true,true),
			-getGyroLimit(), getGyroLimit());
	pitchAttitudeOutput = LIMIT_MIN_MAX_VALUE(
			pidCalculation(&pitchAttitudePidSettings, getPitch(),true,true,true),
			-getGyroLimit(), getGyroLimit());
	yawAttitudeOutput =
			LIMIT_MIN_MAX_VALUE(
					pidCalculation(&yawAttitudePidSettings, yawTransform(getYaw()),true,true,true),
					-getGyroLimit(), getGyroLimit());

	_DEBUG(DEBUG_ATTITUDE_PID_OUTPUT,
			"(%s-%d) attitude pid output: roll=%.5f, pitch=%.5f, yaw=%.5f\n",
			__func__, __LINE__, rollAttitudeOutput, pitchAttitudeOutput,
			yawAttitudeOutput);
}

/**
 * get the output of angular velocity PID controler
 *
 * @param rollRateOutput
 * 		output of roll angular velocity PID controler
 *
 * @param pitchRateOutput
 * 		output of pitch angular velocity PID controler
 *
 * @param yawRateOutput
 * 		output of yaw angular velocity PID controler
 */
void getRatePidOutput(float *rollRateOutput, float *pitchRateOutput,
		float *yawRateOutput) {

	setPidSp(&rollRatePidSettings, rollAttitudeOutput);
	setPidSp(&pitchRatePidSettings, pitchAttitudeOutput);
	setPidSp(&yawRatePidSettings, yawAttitudeOutput);
	*rollRateOutput = pidCalculation(&rollRatePidSettings, getRollGyro(),true,true,true);
	*pitchRateOutput = pidCalculation(&pitchRatePidSettings, getPitchGyro(),true,true,true);
	*yawRateOutput = pidCalculation(&yawRatePidSettings, getYawGyro(),true,true,true);

	_DEBUG(DEBUG_RATE_PID_OUTPUT,
			"(%s-%d) rate pid output: roll=%.5f, pitch=%.5f, yaw=%.5f\n",
			__func__, __LINE__, *rollRateOutput, *pitchRateOutput,
			*yawRateOutput);
}

/**
 *  this function controls motors by PID output
 *
 * @param
 * 		void
 *
 * @return 
 *		void
 *
 */
void motorControler() {

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
	float altThrottleOffset = 0.f;
	float slopThrottleOffset = 1.f;
	float centerThrottle = 0.f;

	altThrottleOffset = (getAltHoldIsReady() && getEnableAltHold())?getThrottleOffsetByAltHold(updateAltHold()):0.f;
	//slopThrottleOffset = getSlopeThrottleOffset();
	centerThrottle = ((float)getThrottlePowerLevel() + altThrottleOffset)*slopThrottleOffset;

	maxLimit = (float) min(
			centerThrottle
					+ getAdjustPowerLeveRange(), getMaxPowerLeve());
	minLimit = (float) max(
			centerThrottle
					- getAdjustPowerLeveRange(), getMinPowerLevel());

	getAttitudePidOutput();
	getRatePidOutput(&rollRateOutput, &pitchRateOutput, &yawRateOutput);

	// rollCa>0
	//    -  CCW2   CW2   +
	//                 X
	//    -   CW1    CCW1  +
	//                H
	//
	// rollCa<0
	//    +  CCW2   CW2    -
	//                 X
	//    +   CW1    CCW1  -
	//            H

	rollCcw1 = rollRateOutput;
	rollCcw2 = -rollRateOutput;
	rollCw1 = -rollRateOutput;
	rollCw2 = rollRateOutput;

	// pitchCa>0
	//    +  CCW2   CW2    +
	//                 X
	//    -  CW1      CCW1   -
	//            H
	//
	//pitchCa<0
	//    -  CCW2   CW2   -
	//                 X
	//    +   CW1   CCW1  +
	//            H

	pitchCcw1 = -pitchRateOutput;
	pitchCcw2 = pitchRateOutput;
	pitchCw1 = -pitchRateOutput;
	pitchCw2 = pitchRateOutput;

	// yawCa>0
	//    +   CCW2   CW2    -
	//                  X
	//    -    CW1   CCW1   +
	//                 H
	//
	// yawCa<0
	//    -  CCW2    CW2  +
	//                 X
	//    +   CW1   CCW1  -
	//                 H

	yawCcw1 = yawRateOutput;
	yawCcw2 = yawRateOutput;
	yawCw1 = -yawRateOutput;
	yawCw2 = -yawRateOutput;

	outCcw1 = centerThrottle
			+ LIMIT_MIN_MAX_VALUE(rollCcw1 + pitchCcw1 + yawCcw1,
					-getPidOutputLimitation(), getPidOutputLimitation());
	outCcw2 = centerThrottle
			+ LIMIT_MIN_MAX_VALUE(rollCcw2 + pitchCcw2 + yawCcw2,
					-getPidOutputLimitation(), getPidOutputLimitation());
	outCw1 = centerThrottle
			+ LIMIT_MIN_MAX_VALUE(rollCw1 + pitchCw1 + yawCw1,
					-getPidOutputLimitation(), getPidOutputLimitation());
	outCw2 = centerThrottle
			+ LIMIT_MIN_MAX_VALUE(rollCw2 + pitchCw2 + yawCw2,
					-getPidOutputLimitation(), getPidOutputLimitation());

	outCcw1 = getMotorGain(
			SOFT_PWM_CCW1) * LIMIT_MIN_MAX_VALUE(outCcw1, minLimit, maxLimit);
	outCcw2 = getMotorGain(
			SOFT_PWM_CCW2) * LIMIT_MIN_MAX_VALUE(outCcw2, minLimit, maxLimit);
	outCw1 = getMotorGain(
			SOFT_PWM_CW1) * LIMIT_MIN_MAX_VALUE(outCw1, minLimit, maxLimit);
	outCw2 = getMotorGain(
			SOFT_PWM_CW2) * LIMIT_MIN_MAX_VALUE(outCw2, minLimit, maxLimit);

	setupCcw1MotorPoewrLevel((unsigned short) outCcw1);
	setupCcw2MotorPoewrLevel((unsigned short) outCcw2);
	setupCw1MotorPoewrLevel((unsigned short) outCw1);
	setupCw2MotorPoewrLevel((unsigned short) outCw2);

}
	
/**
 *  this function controls motors while flipping
 *
 * @param
 * 		void
 *
 * @return 
 *		void
 *
 */
void motorControlerFlipping() {

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
	float centerThrottle = 0.f;
	short invert=1;
	static struct timeval tv_last;
	static float power=0;
	struct timeval tv;

        gettimeofday(&tv, NULL);

	if(GET_USEC_TIMEDIFF(tv,tv_last) < getFlipDelay()*1000000){
		setFlippingStep(0);
		setFlippingFlag(FLIP_NONE);
		motorControler();
		return;
	}

	if(getFlippingStep()==1){
		power=getFlipPower();
		setFlippingStep(2);
	}
	
	if(getZGravity()<0){
		setFlippingStep(3);
	}else{

		if((getFlippingStep()==3)&&(getRoll()<=10.f)){
			setFlippingStep(0);
			setFlippingFlag(FLIP_NONE);
			power=0.f;
			gettimeofday(&tv_last, NULL);
			motorControler();
			return;
		}
	}

	centerThrottle = (float)getThrottlePowerLevel() ;
	maxLimit = (float) getMaxPowerLeve();
	minLimit = (float) getMinPowerLevel();
	power++;

	setPidSp(&rollAttitudePidSettings,
			LIMIT_MIN_MAX_VALUE(0.f, -getAngularLimit(),
			getAngularLimit()));
	setPidSp(&pitchAttitudePidSettings,
		LIMIT_MIN_MAX_VALUE(0.f, -getAngularLimit(),
		getAngularLimit()));
	setPidSp(&yawAttitudePidSettings,
		LIMIT_MIN_MAX_VALUE(0.f, -getAngularLimit(),
		getAngularLimit()));

	if(getFlippingFlag()&(FLIP_LEFT|FLIP_RIGHT)){

		setPidSp(&rollRatePidSettings, LIMIT_MIN_MAX_VALUE(
						pidCalculation(&rollAttitudePidSettings, getRoll(),false,false,false),-getGyroLimit(), getGyroLimit()));
		rollRateOutput = ((getFlippingFlag()&FLIP_LEFT)?-power:power)+ pidCalculation(&rollRatePidSettings, getRollGyro(),false,false,false);
		
	}else{
		setPidSp(&rollRatePidSettings, LIMIT_MIN_MAX_VALUE(
						pidCalculation(&rollAttitudePidSettings, getRoll(),true,true,true),-getGyroLimit(), getGyroLimit()));
		rollRateOutput = pidCalculation(&rollRatePidSettings, getRollGyro(),true,true,true);
	}

	if(getFlippingFlag()&(FLIP_FRONT|FLIP_BACK)){

		setPidSp(&pitchRatePidSettings, LIMIT_MIN_MAX_VALUE(
						pidCalculation(&pitchAttitudePidSettings, getPitch(),false,false,false),-getGyroLimit(), getGyroLimit()));
		pitchRateOutput = ((getFlippingFlag()&FLIP_BACK)?-power:power)+ pidCalculation(&pitchRatePidSettings, getPitchGyro(),false,false,false);
		
	}else{
		setPidSp(&pitchRatePidSettings, LIMIT_MIN_MAX_VALUE(
						pidCalculation(&pitchAttitudePidSettings, getPitch(),true,true,true),-getGyroLimit(), getGyroLimit()));
		pitchRateOutput = pidCalculation(&pitchRatePidSettings, getPitchGyro(),true,true,true);
	}


	setPidSp(&yawRatePidSettings, LIMIT_MIN_MAX_VALUE(
						pidCalculation(&yawAttitudePidSettings, yawTransform(getYaw()),true,true,true),
						-getGyroLimit(), getGyroLimit()));
	yawRateOutput = pidCalculation(&yawRatePidSettings, getYawGyro(),true,true,true);

	// rollCa>0
	//    -  CCW2   CW2   +
	//                 X
	//    -   CW1    CCW1  +
	//                H
	//
	// rollCa<0
	//    +  CCW2   CW2    -
	//                 X
	//    +   CW1    CCW1  -
	//            H

	rollCcw1 = rollRateOutput;
	rollCcw2 = -rollRateOutput;
	rollCw1 = -rollRateOutput;
	rollCw2 = rollRateOutput;

	// pitchCa>0
	//    +  CCW2   CW2    +
	//                 X
	//    -  CW1      CCW1   -
	//            H
	//
	//pitchCa<0
	//    -  CCW2   CW2   -
	//                 X
	//    +   CW1   CCW1  +
	//            H

	pitchCcw1 = -pitchRateOutput;
	pitchCcw2 = pitchRateOutput;
	pitchCw1 = -pitchRateOutput;
	pitchCw2 = pitchRateOutput;

	// yawCa>0
	//    +   CCW2   CW2    -
	//                  X
	//    -    CW1   CCW1   +
	//                 H
	//
	// yawCa<0
	//    -  CCW2    CW2  +
	//                 X
	//    +   CW1   CCW1  -
	//                 H

	yawCcw1 = yawRateOutput;
	yawCcw2 = yawRateOutput;
	yawCw1 = -yawRateOutput;
	yawCw2 = -yawRateOutput;

	if(getZGravity()<0){
		invert=-1.f;
	}else{
		invert=1;
	}

	outCcw1 = LIMIT_MIN_MAX_VALUE(centerThrottle + invert * (rollCcw1 + pitchCcw1 + yawCcw1),minLimit,maxLimit);
	outCcw2 = LIMIT_MIN_MAX_VALUE(centerThrottle + invert * (rollCcw2 + pitchCcw2 + yawCcw2),minLimit,maxLimit);
	outCw1 = LIMIT_MIN_MAX_VALUE(centerThrottle  + invert * (rollCw1 + pitchCw1 + yawCw1),minLimit,maxLimit);
	outCw2 = LIMIT_MIN_MAX_VALUE(centerThrottle + invert * (rollCw2 + pitchCw2 + yawCw2),minLimit,maxLimit);

	setupCcw1MotorPoewrLevel((unsigned short) outCcw1);
	setupCcw2MotorPoewrLevel((unsigned short) outCcw2);
	setupCw1MotorPoewrLevel((unsigned short) outCw1);
	setupCw2MotorPoewrLevel((unsigned short) outCw2);
}


	
/**
 *  get throttle offset for slope
 *
 * @param
 * 		void
 *
 * @return
 *		offset
 *
 */
float getSlopeThrottleOffset() {

	float offset = 1.f;

	if (getZGravity() <= 0.0) {
		//attitude is inverted or vertical
		offset = 1.f;
	} else {
		offset=(2.f-getZGravity())*SlopeThrottleOffsetGain;
	}

	//_DEBUG(DEBUG_NORMAL,"getZGravity=%f\n",getZGravity());
	//_DEBUG(DEBUG_NORMAL,"getSlopeThrottleOffset=%f\n",offset);
	
	return offset;
}

/**
 * quadcopter will record the yaw attitude before flying, this value will become a center point for yaw PID attitude controler
 *
 * @param point
 * 		value
 *
 * @return
 *		offset
 *
 */
void setYawCenterPoint(float point) {

	float yawCenterPoint1 = point;
	if (yawCenterPoint1 > 180.0) {
		yawCenterPoint1 = yawCenterPoint1 - 360.0;
	} else if (yawCenterPoint1 < -180.0) {
		yawCenterPoint1 = yawCenterPoint1 + 360.0;
	}
	yawCenterPoint = yawCenterPoint1;
}

/**
 * get center point ofr yaw
 *
 * @param
 * 		void
 *
 * @return
 *		value
 *
 */
float getYawCenterPoint() {

	return yawCenterPoint;
}

/**
 * transform yaw value by YawCenterPoint
 *
 * @param originPoint
 *               a real yaw value
 *
 * @return
 *		the yaw value after transform
 */
float yawTransform(float originPoint) {

	float output = originPoint - yawCenterPoint;
	if (output > 180.0) {
		output = output - 360.0;
	} else if (output < -180.0) {
		output = output + 360.0;
	}
	return output;
}

/**
 * record start altitude
 *
 * @param v
 *               start altitude
 *
 * @return
 *		void
 */
void setAltStartPoint(float v){
	altStartPoint=v;
}

/**
 * get start altitude
 *
 * @param 
 *               void
 *
 * @return
 *		start altitude
 */
float getAltStartPoint(){
	return altStartPoint;
}

/**
 * set a value to limit the PID output of attitude
 *
 * @param limitation
 * 		the period of adjusting motor
 *
 * @return
 *		void
 *
 */
void setGyroLimit(float limitation) {

	gyroLimit = limitation;
}

/**
 * get the limitation of PID output of attitude
 *
 * @param
 * 		void
 *
 * @return
 *		 the limitation of PID output of attitude
 *
 */
float getGyroLimit() {

	return gyroLimit;
}

/**
 * set a value to indicate the period of adjusting motor
 *
 * @param period
 * 		the period of adjusting motor
 *
 * @return
 *		void
 *
 */
void setAdjustPeriod(unsigned short period) {

	adjustPeriod = period;
}

/**
 * get the period of adjusting motor
 *
 * @param
 * 		void
 *
 * @return
 *		 the period of adjusting motor
 *
 */
unsigned short getAdjustPeriod() {

	return adjustPeriod;
}

/**
 * set a value to limit the maximum of angular which is got from remote controler
 *
 * @param angular
 *               the limitation
 *
 * @return
 *			void
 *
 */
void setAngularLimit(float angular) {

	angularLimit = angular;
}

/**
 * get the maxumum of angular that  your quadcopter can get
 *
 * @param
 *		whetjer update altHold offset or not
 *
 * @return
 *		the limitation of angular
 */
float getAngularLimit() {

	return angularLimit;
}

/**
 * set the limitation of output of altiude PID controler
 *
 * @param
 *		limitation
 *
 * @return
 *		void
 */
void setAltitudePidOutputLimitation(float v) {

	altitudePidOutputLimitation = v;
}

/**
 * get the limitation of output of altiude PID controler
 *
 * @param
 *		void
 *
 * @return
 *		limitation
 */
float getAltitudePidOutputLimitation(void) {

	return altitudePidOutputLimitation;
}

/**
 * get output from altitude pid controler
 *
 * @param
 *		void
 *
 * @return
 *		void
 */
void getAltHoldAltPidOutput() {

	altHoltAltOutput =
			LIMIT_MIN_MAX_VALUE(
					pidCalculation(&altHoldAltSettings, max(getCurrentAltHoldAltitude()-getAltStartPoint(),0.f),true,true,true),
					-getAltitudePidOutputLimitation(),
					getAltitudePidOutputLimitation());
	
	//_DEBUG(DEBUG_NORMAL,"getPidSp(&altHoldAltSettings)=%f\n",getPidSp(&altHoldAltSettings));
	//_DEBUG(DEBUG_NORMAL,"getCurrentAltHoldAltitude=%f,getAltStartPoint=%f\n",getCurrentAltHoldAltitude(),getAltStartPoint());
	//_DEBUG(DEBUG_NORMAL,"altHoltAltOutput=%f\n",altHoltAltOutput);
}

/**
 * get output from speed pid controler
 *
 * @param
 *		output
 *
 * @return
 *		void
 */
void getAltHoldSpeedPidOutput(float *altHoldSpeedOutput) {

	setPidSp(&altHoldlSpeedSettings, altHoltAltOutput);
	*altHoldSpeedOutput = pidCalculation(&altHoldlSpeedSettings,
			getCurrentAltHoldSpeed(),true,true,true);
	//_DEBUG(DEBUG_NORMAL,"getCurrentAltHoldSpeed=%f\n",getCurrentAltHoldSpeed());
	//_DEBUG(DEBUG_NORMAL,"altHoldSpeedOutput=%f\n",*altHoldSpeedOutput);
}

/**
 * get throttle offset by altHold mechanism
 *
 * @param
 *		void
 *
 * @return
 *		void
 */
float getThrottleOffsetByAltHold(bool updateAltHoldOffset) {

	static float output = 0.f;

	if (updateAltHoldOffset) {

		getAltHoldAltPidOutput();
		getAltHoldSpeedPidOutput(&output);
		output = LIMIT_MIN_MAX_VALUE(output, -maxThrottleOffset,
				maxThrottleOffset);
	}
	//_DEBUG(DEBUG_NORMAL,"output =%f\n",output);
	return output;
}

/**
 *  get flip flag
 *
 * @param
 * 		void
 *
 * @return 
 *		flag
 *
 */
unsigned char getFlippingFlag(){
	return flippingFlag;
}

/**
 *  set flip flag
 *
 * @param val
 * 		flag
 *
 * @return 
 *		void
 *
 */
void setFlippingFlag(unsigned char val){
	flippingFlag=val;
}

/**
 *  get flip step
 *
 * @param 
 * 		void
 *
 * @return 
 *		step
 *
 */
unsigned char getFlippingStep(){
	return flipStep;
}

/**
 *  set flip step
 *
 * @param val
 * 		step
 *
 * @return 
 *		void
 *
 */
void setFlippingStep(unsigned char val){
	flipStep=val;
}

/**
 *  get flip step
 *
 * @param 
 * 		void
 *
 * @return 
 *		step
 *
 */
bool getFlippingIsEnable(){
	return flipIsEnable;
}

/**
 *  Enable or disable flipping mechanism
 *
 * @param val
 * 		bool
 *
 * @return 
 *		void
 *
 */
void setFlippingIsEnable(bool val){
	flipIsEnable=val;
}

/**
 * setup flipping threahold
 *
 * @param v
 * 		flipping threahold
 *
 * @return 
 *		void
 *
 */
void setFlipThreadHold(unsigned char v){
	flipThreshold=v;
}

/**
 * get flipping threahold
 *
 * @param v
 * 		void
 *
 * @return 
 *		flipping threahold
 *
 */
unsigned char getFlipThreadHold(){
	return flipThreshold;
}

/**
 * setup flipping delay
 *
 * @param v
 * 		flipping delay
 *
 * @return 
 *		void
 *
 */
void setFlipDelay(unsigned char v){
	flipDelay=v;
}

/**
 * get flipping delay
 *
 * @param v
 * 		void
 *
 * @return 
 *		flipping delay
 *
 */
unsigned char getFlipDelay(){
	return flipDelay;
}

/**
 * setup flipping power
 *
 * @param v
 * 		flipping power
 *
 * @return 
 *		void
 *
 */
void setFlipPower(unsigned short v){
	flipPower=v;
}

/**
 * get flipping power
 *
 * @param v
 * 		void
 *
 * @return 
 *		flipping power
 *
 */
unsigned short getFlipPower(){
	return flipPower;
}

