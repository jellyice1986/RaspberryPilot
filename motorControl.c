
#include <stdlib.h>
#include <stdio.h>
#include "commonLib.h"
#include "pca9685.h"
#include "motorControl.h"

#define DEFAULT_ADJUST_POWER_RANGE 500  
#define DEFAULT_PID_ADJUST_POWER_LIMIT 500
#define PWM_DUTY_CYCLE  500				//HZ
#define PWM_MAX_DC 500 					//HZ   2ms
#define PWM_MIN_DC 1000 				//HZ  1ms
#define MAX_POWER_LEVEL 4096    		//4096*(PWM_DUTY_CYCLE/PWM_MAX_DC)
#define MIN_POWER_LEVEL 2048 			//(4096*(PWM_DUTY_CYCLE/PWM_MAX_DC))

static unsigned short motorPowerLevel_CW1;
static unsigned short motorPowerLevel_CW2;
static unsigned short motorPowerLevel_CCW1;
static unsigned short motorPowerLevel_CCW2;
static unsigned short ThrottlePowertlevel;
static unsigned short adjustPowerLevelRange=DEFAULT_ADJUST_POWER_RANGE;
static unsigned short pidAdjustPowerLimit=DEFAULT_PID_ADJUST_POWER_LIMIT;
static float motor_0_gain;
static float motor_1_gain;
static float motor_2_gain;
static float motor_3_gain;


/**
* init motors status
*
* @param 
* 		void
*
* @return 
*		void
*
*/
void motorInit() {

	resetPca9685();
	setPWMFreq(PWM_DUTY_CYCLE);
	setupAllMotorPoewrLevel(MIN_POWER_LEVEL, MIN_POWER_LEVEL, MIN_POWER_LEVEL,
			MIN_POWER_LEVEL);
	setThrottlePowerLevel(MIN_POWER_LEVEL);
}


/**
* Setup all motors power level
*
* @param CW1
*		power for CW1
* @param CW2
*		power for CW2
* @param CCW1
*		power for CCW1
* @param CCW2
*		power for CCW2
*
* @return 
*		void
*
*/
void setupAllMotorPoewrLevel(unsigned short CW1, unsigned short CW2,
		unsigned short CCW1, unsigned short CCW2) {
		
	setupCcw1MotorPoewrLevel(CCW1);
	setupCcw2MotorPoewrLevel(CCW2);
	setupCw1MotorPoewrLevel(CW1);
	setupCw2MotorPoewrLevel(CW2);
}

/**
* Set power level for motor CCW1
*
* @param CCW1
*		Set power for CCW1
*
* @return 
*		void
*
*/
void setupCcw1MotorPoewrLevel(unsigned short CCW1) {

	motorPowerLevel_CCW1 = CCW1;
	setPWM(SOFT_PWM_CCW1, motorPowerLevel_CCW1);
}

/**
* Set power level for motor CCW2
*
* @param CCW2
*		Set power for CCW2
*
* @return 
*		void
*
*/
void setupCcw2MotorPoewrLevel(unsigned short CCW2) {

	motorPowerLevel_CCW2 = CCW2;
	setPWM(SOFT_PWM_CCW2, motorPowerLevel_CCW2);
}

/**
* Set power level for motor CW1
*
* @param CW1
*		Set power for CW1
*
* @return 
*		void
*
*/
void setupCw1MotorPoewrLevel(unsigned short CW1) {

	motorPowerLevel_CW1 = CW1;
	setPWM(SOFT_PWM_CW1, motorPowerLevel_CW1);
}

/**
* Set power level for motor CW2
*
* @param CW2
*		Set power for CW2
*
* @return 
*		void
*
*/
void setupCw2MotorPoewrLevel(unsigned short CW2) {

	motorPowerLevel_CW2 = CW2;
	setPWM(SOFT_PWM_CW2, motorPowerLevel_CW2);
}

/**
* get power level of motor CW1
*
* @param 
*		void
*
* @return
*		 power of CW1
*
*/
unsigned short getMotorPowerLevelCW1() {
	return motorPowerLevel_CW1;
}

/**
* get power level of motor CW2
*
* @param 
*		void
*
* @return
*		 power of CW2
*
*/
unsigned short getMotorPowerLevelCW2() {
	return motorPowerLevel_CW2;
}

/**
* get power level of motor CCW1
*
* @param 
*		void
*
* @return
*		 power of CCW1
*
*/
unsigned short getMotorPowerLevelCCW1() {
	return motorPowerLevel_CCW1;
}

/**
* get power level of motor CCW2
*
* @param 
*		void
*
* @return
*		 power of CCW2
*
*/
unsigned short getMotorPowerLevelCCW2() {
	return motorPowerLevel_CCW2;
}

/**
* get current power level of throttle, motorPowerLevel_XXX= ThrottlePowertlevel+pid output
*
* @param 
*		void
*
* @return
*		 power of throttle
*
*/
unsigned short getThrottlePowerLevel() {

	return ThrottlePowertlevel;
}

/**
* set current power level of throttle, this value is got from remote controler
*
* @param level
*		 power of throttle
*
* @return
*		 void
*
*/
void setThrottlePowerLevel(unsigned short level) {

	ThrottlePowertlevel = level;
}

/*
* Get minimum power level
*
* @param 
*		 void
*
* @return 
*	minimum power level
*
*/
unsigned short getMinPowerLevel() {

	return MIN_POWER_LEVEL;
}

/*
*Get maximum power level
*
* @param 
*		 void
*
* @return 
*	maximum power level
*/
unsigned short getMaxPowerLeve() {

	return MAX_POWER_LEVEL;
}

/*
*	Get the range of adjusting of power level , the range of adjusting of mtor is as below
*	maximum : min(getThrottlePowerLevel + getAdjustPowerLeveRange,getMaxPowerLeve)
*	minimum : max(getThrottlePowerLevel - getAdjustPowerLeveRange,getMinPowerLevel)
*
* @param 
*		 void
*
* @return 
*		limitation of adjusting of power level
*
*/
unsigned short getAdjustPowerLeveRange() {

	return adjustPowerLevelRange;
}

/*
*	the range of adjusting of power level , the range of adjusting of mtor is as below
*	maximum : min(getThrottlePowerLevel + getAdjustPowerLeveRange,getMaxPowerLeve)
*	minimum : max(getThrottlePowerLevel - getAdjustPowerLeveRange,getMinPowerLevel)
*
* @param 
*		limitation of adjusting power level
*
* @return 
*		void

*/
void setAdjustPowerLeveRange(int v) {

	adjustPowerLevelRange = (unsigned short) v;
}

/*
* Get the range of adjusting of power level that PID controler can adjust
*
* @param 
*		void
*
* @return 
*		limitation of PID controler output
*
*/
unsigned short getPidAdjustPowerLimit() {

	return pidAdjustPowerLimit;
}

/*
* Set the range of adjusting of power level that PID controler can adjust
*
* @param 
*		limitation of PID controler output
*
* @return 
*		void
*/
void setPidAdjustPowerLimit(int v) {

	pidAdjustPowerLimit = (unsigned short) v;
}



/**
 * set motor gain
 *
 * @param index
 *  		motor index:
 *  		2  CCW2     CW2   3
 *		         	X
 *	   	1   CW1      CCW1  0
 *	 	     		H
 *
 * @param value
 *		motor gain
 * 
 * @return
 *		void
 *
 */
void setMotorGain(unsigned char index, float value) {

	switch (index) {
	case SOFT_PWM_CCW1:
		motor_0_gain = value;
		break;
	case SOFT_PWM_CW1:
		motor_1_gain = value;
		break;
	case SOFT_PWM_CCW2:
		motor_2_gain = value;
		break;
	case SOFT_PWM_CW2:
		motor_3_gain = value;
		break;
	}
}

/**
 * get motor gain
 *
 * @param index
 *  		motor index:
 *  		2  CCW2     CW2    3
 *		           	X
 *		1   CW1      CCW1  0
 *		       	H
 *
 * @return
 *  		gain of motor
 *
 */
float getMotorGain(unsigned char index) {
	float value = 1;
	switch (index) {
	case SOFT_PWM_CCW1:
		value = motor_0_gain;
		break;
	case SOFT_PWM_CW1:
		value = motor_1_gain;
		break;
	case SOFT_PWM_CCW2:
		value = motor_2_gain;
		break;
	case SOFT_PWM_CW2:
		value = motor_3_gain;
		break;
	}
	return value;
}


