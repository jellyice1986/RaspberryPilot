
#include <stdlib.h>
#include <stdio.h>

#include "pca9685.h"
#include "motorControl.h"

#define DEFAULT_ADJUST_POWER_RANGE 500  //total adjusted range
#define DEFAULT_ADJUST_POWER_LIMIT 500  //adjusted limitation per adjustation
#define PWM_DUTY_CYCLE  490//HZ 
#define PWM_MAX_DC 500 //HZ   2ms
#define PWM_MIN_DC 1000 //HZ  1ms
#define MAX_POWER_LEVEL 4014    //4096*(490 HZ/500 HZ)
#define MIN_POWER_LEVEL 2014 //(4096*(490 HZ/1000 HZ))+7


/*
* Power level is defined by PCA9685, please check pca9685.h 
*/
static unsigned short motorPowerLevel_CW1;
static unsigned short motorPowerLevel_CW2;
static unsigned short motorPowerLevel_CCW1;
static unsigned short motorPowerLevel_CCW2;
static unsigned short ThrottlePowertlevel;
static unsigned short adjustPowerLevelRange;
static unsigned short adjustPowerLimit;
static float motor_0_gain;
static float motor_1_gain;
static float motor_2_gain;
static float motor_3_gain;


/**
* Init paramtes and states of motors 
*/
void motorInit() {

	pca9685Init();
	setAdjustPowerLeveRange(DEFAULT_ADJUST_POWER_RANGE);
	setAdjustPowerLimit(DEFAULT_ADJUST_POWER_LIMIT);
	setPWMFreq(PWM_DUTY_CYCLE);
	setupAllMotorPoewrLevel(MIN_POWER_LEVEL, MIN_POWER_LEVEL, MIN_POWER_LEVEL,
			MIN_POWER_LEVEL);
	setThrottlePowerLevel(MIN_POWER_LEVEL);
}


/**
*	Set all motor power level
* @param CW1
*		Set power for CW1
* @param CW2
*		Set power for CW2
* @param CCW1
*		Set power for CCW1
* @param CCW2
*		Set power for CCW2
*
*/
void setupAllMotorPoewrLevel(unsigned short CW1, unsigned short CW2,
		unsigned short CCW1, unsigned short CCW2) {
		
	setupCcw1MotorPoewrLevel(CCW1);
	setupCcw2MotorPoewrLevel(CCW2);
	setupCw1MotorPoewrLevel(CW1);
	setupCw2MotorPoewrLevel(CW2);
//	printf("MAX_POWER_LEVEL=%d, MIN_POWER_LEVEL=%d\n",MAX_POWER_LEVEL,MIN_POWER_LEVEL);
//	printf(" motorPowerLevel_CCW1 %d, motorPowerLevel_CCW2 %d, motorPowerLevel_CW1 %d , motorPowerLevel_CW2 %d\n", 
	//           motorPowerLevel_CCW1, motorPowerLevel_CCW2, motorPowerLevel_CW1, motorPowerLevel_CW2);

}

/**
*	Set power level for motor CCW1
* @param CCW1
*		Set power for CCW1
*
*/
void setupCcw1MotorPoewrLevel(unsigned short CCW1) {

	//if(motorPowerLevel_CCW1!=CCW1)
		motorPowerLevel_CCW1 = CCW1;
	//else{
		//printf("CCW1 no change\n");
	//}
	setPWM(SOFT_PWM_CCW1, motorPowerLevel_CCW1);
}

/**
*	Set power level for motor CCW2
* @param CCW2
*		Set power for CCW2
*
*/
void setupCcw2MotorPoewrLevel(unsigned short CCW2) {

	//if(motorPowerLevel_CCW2!=CCW2)
		motorPowerLevel_CCW2 = CCW2;
	//else{
		//printf("CCW2 no change\n");
	//}
	setPWM(SOFT_PWM_CCW2, motorPowerLevel_CCW2);
}

/**
*	Set power level for motor CW1
* @param CW1
*		Set power for CW1
*
*/
void setupCw1MotorPoewrLevel(unsigned short CW1) {

	//if(motorPowerLevel_CW1!=CW1)
		motorPowerLevel_CW1 = CW1;
	//else{
		//printf("CW1 no change\n");
	//}
	setPWM(SOFT_PWM_CW1, motorPowerLevel_CW1);
}

/**
*	Set power level for motor CW2
* @param CW2
*		Set power for CW2
*
*/
void setupCw2MotorPoewrLevel(unsigned short CW2) {

	//if(motorPowerLevel_CW2!=CW2)
		motorPowerLevel_CW2 = CW2;
	//else{
			//printf("CW2 no change\n");
	//}
	setPWM(SOFT_PWM_CW2, motorPowerLevel_CW2);
}

/**
*	get power level of motor CW1
* @return
*		 power of CW1
*
*/
unsigned short getMotorPowerLevelCW1() {
	return motorPowerLevel_CW1;
}

/**
*	get power level of motor CW2
* @return
*		 power of CW2
*
*/
unsigned short getMotorPowerLevelCW2() {
	return motorPowerLevel_CW2;
}

/**
*	get power level of motor CCW1
* @return
*		 power of CCW1
*
*/
unsigned short getMotorPowerLevelCCW1() {
	return motorPowerLevel_CCW1;
}

/**
*	get power level of motor CCW2
* @return
*		 power of CCW2
*
*/
unsigned short getMotorPowerLevelCCW2() {
	return motorPowerLevel_CCW2;
}

/**
*	get current power level of throttle
* @return
*		 power of throttle
*
*/
unsigned short getThrottlePowerLevel() {
	return ThrottlePowertlevel;
}

/**
*	set current power level of throttle, this value is got from remote controler
* @param level
*		 power of throttle
*
*/
void setThrottlePowerLevel(unsigned short level) {
	ThrottlePowertlevel = level;
}

/*
*	Get minimum power level
* @return 
*	minimum power level
*/
unsigned short getMinPowerLevel() {
	return MIN_POWER_LEVEL;
}

/*
*	Get maximum power level
* @return 
*	maximum power level
*/
unsigned short getMaxPowerLeve() {
	return MAX_POWER_LEVEL;
}

/*
*	Get limitation for adjusting power level , the power range for adjusting motor is as below
*	maximum : getThrottlePowerLevel + getAdjustPowerLeveRange
*	minimum : getThrottlePowerLevel - getAdjustPowerLeveRange
*
* @return 
*	limitation of adjusting power level
*/
unsigned short getAdjustPowerLeveRange() {
	return adjustPowerLevelRange;
}

/*
*	Set limitation for adjusting power level , the power range for adjusting motor is as below
*	maximum : getThrottlePowerLevel + getAdjustPowerLeveRange
*	minimum : getThrottlePowerLevel - getAdjustPowerLeveRange
*
* @param 
*	limitation of adjusting power level
*/
void setAdjustPowerLeveRange(int v) {
	adjustPowerLevelRange = (unsigned short) v;
}

/*
*	Get the limitation value per adjustment,
*
* @return 
*	limitation value per adjustment
*/
unsigned short getAdjustPowerLimit() {
	return adjustPowerLimit;
}

/*
*	Set the limitation value per adjustment,
*
* @param 
*	limitation value per adjustment
*/
void setAdjustPowerLimit(int v) {
	adjustPowerLimit = (unsigned short) v;
}



/**
 *  set motor gain
 *  @param index
 *  			index of motor
 *
 *  		    2  CCW2    CW2   3
 *		            	X
 *		    	1   CW1    CCW1  0
 *		         		H
 *
 *  @param value
 *  			gain of motor
 */
void setMotorGain(short index, float value) {
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
 *  get motor gain
 *  @param index
 *  		index of motor
 *
 *  		2  CCW2    CW2   3
 *		           	X
 *		    1   CW1    CCW1  0
 *		       		H
 *
 *  @param value
 *  			gain of motor
 */
float getMotorGain(short index) {
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


