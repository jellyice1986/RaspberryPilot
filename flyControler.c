
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
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

static float getThrottleOffset();
static void getAttitudePidOutput();
void getAltHoldAltPidOutput ();
void getAltHoldSpeedPidOutput(	float *altHoldSpeedOutput);
float getThrottleOffsetByAltHold(bool updateAltHoldOffset);


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


/**
* Init paramtes and states for flyControler 
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void flyControlerInit(){

	setLeaveFlyControlerFlag(false);
	disenableFlySystem();
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
	altHoltAltOutput=0;
	maxThrottleOffset=200;
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
void setLeaveFlyControlerFlag(bool v){
	leaveFlyControler=v;
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
bool getLeaveFlyControlerFlag(){
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

	rollAttitudeOutput = LIMIT_MIN_MAX_VALUE(pidCalculation(&rollAttitudePidSettings, getRoll(),true),
			-getGyroLimit(), getGyroLimit());
	pitchAttitudeOutput = LIMIT_MIN_MAX_VALUE(pidCalculation(&pitchAttitudePidSettings, getPitch(),true),
			-getGyroLimit(), getGyroLimit());
	yawAttitudeOutput = LIMIT_MIN_MAX_VALUE(pidCalculation(&yawAttitudePidSettings, yawTransform(getYaw()),true),
			-getGyroLimit(), getGyroLimit());
	
	_DEBUG(DEBUG_ATTITUDE_PID_OUTPUT,"(%s-%d) attitude pid output: roll=%.5f, pitch=%.5f, yaw=%.5f\n",__func__,__LINE__,rollAttitudeOutput,pitchAttitudeOutput,yawAttitudeOutput);
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
	*rollRateOutput = pidCalculation(&rollRatePidSettings, getRollGyro(),true);
	*pitchRateOutput = pidCalculation(&pitchRatePidSettings, getPitchGyro(),true);
	*yawRateOutput = pidCalculation(&yawRatePidSettings, getYawGyro(),true);
	
	_DEBUG(DEBUG_RATE_PID_OUTPUT,"(%s-%d) rate pid output: roll=%.5f, pitch=%.5f, yaw=%.5f\n",__func__,__LINE__,*rollRateOutput,*pitchRateOutput,*yawRateOutput);
}



/**
 *  this function adjust motors by PID output
 *
 * @param
 * 		whether update altHold offset or not
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
	float throttleOffset = 0.f;

	if(getAltHoldIsReady() && getEnableAltHold()){
		throttleOffset=getThrottleOffsetByAltHold(updateAltHold());
	}else{
		//have to check whether it is useful or not
		//throttleOffset=getThrottleOffset();
	}

	maxLimit = (float) min(
			(getThrottlePowerLevel() + throttleOffset) + getAdjustPowerLeveRange(),
			getMaxPowerLeve());
	minLimit = (float) max(
			(getThrottlePowerLevel() + throttleOffset) - getAdjustPowerLeveRange(),
			getMinPowerLevel());

	getAttitudePidOutput();
	getRatePidOutput(&rollRateOutput, &pitchRateOutput, &yawRateOutput);

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

	// yawCa>0
	//    +   CCW2   CW2    -
	//                 X
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

	outCcw1 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCcw1+pitchCcw1+yawCcw1,-getPidOutputLimitation(),getPidOutputLimitation());
	outCcw2 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCcw2+pitchCcw2+yawCcw2,-getPidOutputLimitation(),getPidOutputLimitation());
	outCw1 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCw1+pitchCw1+yawCw1,-getPidOutputLimitation(),getPidOutputLimitation());
	outCw2 =
			((float) getThrottlePowerLevel()+ throttleOffset) + LIMIT_MIN_MAX_VALUE(rollCw2+pitchCw2+yawCw2,-getPidOutputLimitation(),getPidOutputLimitation());

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

}

/**
*  calculate throttle offset,  keep the height by applying the offset to motors
*
* @param 
* 		void   
*
* @return 
*		offset
*		
*/
#if 0
float getThrottleOffset() {

	float offset = 0.f;
	
	if (getZAxisSlope() <= 0.0) {
		//inverted or vertical
		offset = 0.0;
	} else {
		offset = (((1.0 - getZAxisSlope()) >= 0.3) ? 0.3 : 1.0 - getZAxisSlope())
				* ((float) (getThrottlePowerLevel() - getMinPowerLevel()));
	}

	return offset;
}
#endif

/**
* quadcopter will record the yaw before flying, this value will be a center point for yaw PID attitude controler
*
* @param point
* 		value   
*
* @return 
*		offset
*		
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
* get center point ofr yaw 
*
* @param 
* 		void   
*
* @return 
*		value
*		
*/
float getYawCenterPoint(){
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
float yawTransform(float originPoint){
	float output=originPoint-yawCenterPoint;
	if(output>180.0){
		output=output-360.0;
	}else if(output<-180.0){
		output=output+360.0;
	}
	return output;
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
* get output from altitude pid controler  
*
* @param 
*		void
*
* @return 
*		void
*/
void getAltHoldAltPidOutput () {
	
	altHoltAltOutput = pidCalculation(&altHoldAltSettings, getCurrentAltHoldAltitude(),true);
	//_DEBUG(DEBUG_NORMAL,"getCurrentAltHoldAltitude=%f\n",getCurrentAltHoldAltitude());
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
void getAltHoldSpeedPidOutput(	float *altHoldSpeedOutput) {

	setPidSp(&altHoldlSpeedSettings, altHoltAltOutput);
	*altHoldSpeedOutput = pidCalculation(&altHoldlSpeedSettings, getCurrentAltHoldSpeed(),true);
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
float getThrottleOffsetByAltHold(bool updateAltHoldOffset){

	static float output=0;

	if(updateAltHoldOffset){
		
		getAltHoldAltPidOutput();
		getAltHoldSpeedPidOutput(&output);
		output=LIMIT_MIN_MAX_VALUE(output,-maxThrottleOffset,maxThrottleOffset);
	}
	//_DEBUG(DEBUG_NORMAL,"output =%f\n",output);
	return output;
}

