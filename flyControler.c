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
#include "attitudeUpdate.h"
#include "pid.h"
#include "altHold.h"
#include "flyControler.h"

#define DEFAULT_ADJUST_PERIOD 1
#define DEFAULT_GYRO_LIMIT 50
#define DEFAULT_ANGULAR_LIMIT 5000

static void getAttitudePidOutput();
static float getThrottleOffsetByAltHold(bool updateAltHoldOffset);
static float getThrottleOffsetByAcceleration(void);
static void getAltHoldAltPidOutput();
static void getAltHoldSpeedPidOutput(float *altHoldSpeedOutput);

pthread_mutex_t controlMotorMutex;

static bool leaveFlyControler;
static float rollAttitudeOutput;
static float pitchAttitudeOutput;
static float yawAttitudeOutput;
static float altHoltAltOutput;
static float yawCenterPoint;
static float maxThrottleOffset;

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
	rollAttitudeOutput = 0.f;
	pitchAttitudeOutput = 0.f;
	yawAttitudeOutput = 0.f;
	altHoltAltOutput = 0.f;
	maxThrottleOffset = 1000.f;

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

	rollAttitudeOutput = pidCalculation(&rollAttitudePidSettings, getRoll(),true,true,true);
	pitchAttitudeOutput = pidCalculation(&pitchAttitudePidSettings, getPitch(),true,true,true);
	yawAttitudeOutput =	pidCalculation(&yawAttitudePidSettings, yawTransform(getYaw()),true,true,true);

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
	*rollRateOutput = pidCalculation(&rollRatePidSettings, getRollGyro(), true,
	true, true);
	*pitchRateOutput = pidCalculation(&pitchRatePidSettings, getPitchGyro(),
	true, true, true);
	*yawRateOutput = pidCalculation(&yawRatePidSettings, getYawGyro(), true,
	true, true);

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
	float accelThrottleOffset = 0.f;
	float altholdThrottleOffset = 0.f;
	float throttleOffset = 0.f;
	float centerThrottle = 0.f;

	altholdThrottleOffset = (getEnableAltHold() && getAltHoldIsReady()) ? getThrottleOffsetByAltHold(updateAltHold()) : 0.f;
	accelThrottleOffset = getThrottleOffsetByAcceleration();
	throttleOffset = altholdThrottleOffset + accelThrottleOffset;

	centerThrottle = (float) getThrottlePowerLevel() + throttleOffset;

	maxLimit = (float) getMaxPowerLeve();
	minLimit = (float) getMinPowerLevel();

	getAttitudePidOutput();
	getRatePidOutput(&rollRateOutput, &pitchRateOutput, &yawRateOutput);

	/*
	 *	 rollCa>0
	 *	    -  CCW2   CW2   +
	 *	            	   X
	 *	    -   CW1    CCW1  +
	 *	            	   F
	 *
	 *	 rollCa<0
	 *	    +  CCW2   CW2    -
	 *	            	   X
	 *	    +   CW1    CCW1  -
	 *	            	   F
	 */
	rollCcw1 = rollRateOutput;
	rollCcw2 = -rollRateOutput;
	rollCw1 = -rollRateOutput;
	rollCw2 = rollRateOutput;
	
	/*
	 *	 pitchCa>0
	 *	    +  CCW2   CW2    +
	 *	            	   X
	 *	    -  CW1      CCW1   -
	 *	           	   F
	 *
	 *	pitchCa<0
	 *	    -  CCW2   CW2   -
	 *	                 X
	 *	    +   CW1   CCW1  +
	 *	           	   F
	 */
	pitchCcw1 = -pitchRateOutput;
	pitchCcw2 = pitchRateOutput;
	pitchCw1 = -pitchRateOutput;
	pitchCw2 = pitchRateOutput;
	
	/*
	 *	 yawCa>0
	 *	    +   CCW2   CW2    -
	 *	            	    X
	 *	    -    CW1   CCW1   +
	 *	                  F
	 *
	 *	 yawCa<0
	 *	    -  CCW2    CW2  +
	 *	            	   X
	 *	    +   CW1   CCW1  -
	 *	            	   F
	 */
	yawCcw1 = yawRateOutput;
	yawCcw2 = yawRateOutput;
	yawCw1 = -yawRateOutput;
	yawCw2 = -yawRateOutput;

	outCcw1 = centerThrottle + (rollCcw1 + pitchCcw1 + yawCcw1);
	outCcw2 = centerThrottle + (rollCcw2 + pitchCcw2 + yawCcw2);
	outCw1 = centerThrottle	+ (rollCw1 + pitchCw1 + yawCw1);
	outCw2 = centerThrottle	+ (rollCw2 + pitchCw2 + yawCw2);

	outCcw1 =  LIMIT_MIN_MAX_VALUE(outCcw1, minLimit, maxLimit);
	outCcw2 =  LIMIT_MIN_MAX_VALUE(outCcw2, minLimit, maxLimit);
	outCw1 =  LIMIT_MIN_MAX_VALUE(outCw1, minLimit, maxLimit);
	outCw2 = LIMIT_MIN_MAX_VALUE(outCw2, minLimit, maxLimit);

	setupCcw1MotorPoewrLevel((unsigned short) outCcw1);
	setupCcw2MotorPoewrLevel((unsigned short) outCcw2);
	setupCw1MotorPoewrLevel((unsigned short) outCw1);
	setupCw2MotorPoewrLevel((unsigned short) outCw2);
#if 0
	_DEBUG(DEBUG_NORMAL,"outCcw1=%d,outCcw2=%d,outCw1=%d,outCw2=%d\n" ,
		(unsigned short)outCcw1,
		(unsigned short)outCcw2,
		(unsigned short)outCw1,
		(unsigned short)outCw2);
#endif
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
 * get output from altitude pid controler
 *
 * @param
 *		void
 *
 * @return
 *		void
 */
void getAltHoldAltPidOutput() {

	altHoltAltOutput =	pidCalculation(&altHoldAltSettings, getCurrentAltHoldAltitude()-getTargetAlt(),true,true,true);
	
	//_DEBUG(DEBUG_NORMAL,"getPidSp(&altHoldAltSettings)=%f\n",getPidSp(&altHoldAltSettings));
	//_DEBUG(DEBUG_NORMAL,"getCurrentAltHoldAltitude=%f,getTargetAlt=%f\n",getCurrentAltHoldAltitude(),getTargetAlt());
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
			getAltholdSpeed(),true,true,true);
	//_DEBUG(DEBUG_NORMAL,"getAltholdSpeed=%f\n",getAltholdSpeed());
	//_DEBUG(DEBUG_NORMAL,"altHoldSpeedOutput=%f, altHoltAltOutput=%f\n",*altHoldSpeedOutput, altHoltAltOutput);
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

	float output = 0.f;

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
 * get throttle offset by acceleration
 *
 * @param
 *		void
 *
 * @return
 *		void
 */
float getThrottleOffsetByAcceleration(void) {

	float output = 0.f;

	setPidSp(&verticalAccelPidSettings, 0.f);
	
	output = LIMIT_MIN_MAX_VALUE(pidCalculation(&verticalAccelPidSettings,
			getVerticalAcceleration(),true,true,true), -maxThrottleOffset,
		maxThrottleOffset);
	
	//_DEBUG(DEBUG_NORMAL,"%s output =%f\n",__func__,output);
	return output;
}

