/******************************************************************************
The motorControl.h in RaspberryPilot project is placed under the MIT license

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

/**
 * motor definition
 *	  2 CCW2  CW2 3
 *			X
 *	  1 CW1  CCW1 0
 *			H
 */
typedef enum {
	SOFT_PWM_CCW1, SOFT_PWM_CW1, SOFT_PWM_CCW2, SOFT_PWM_CW2
} MOTOR_INDEX;

#define DEFAULT_ADJUST_POWER_RANGE 500  
#define DEFAULT_PID_OUTPUT_LIMITATION 500
#define PWM_DUTY_CYCLE  490				//HZ
#define PWM_MAX_DC 500 					//HZ   2ms
#define PWM_MIN_DC 1000 				//HZ  1ms
#define MAX_POWER_LEVEL 4014    		//4096*(PWM_DUTY_CYCLE/PWM_MAX_DC)
#define MIN_POWER_LEVEL 2014 			//(4096*(PWM_DUTY_CYCLE/PWM_MIN_DC))+7

void motorInit();
void setupAllMotorPoewrLevel(unsigned short CW1, unsigned short CW2,
		unsigned short CCW1, unsigned short CCW2);
unsigned short getMotorPowerLevelCW1();
unsigned short getMotorPowerLevelCW2();
unsigned short getMotorPowerLevelCCW1();
unsigned short getMotorPowerLevelCCW2();
void setupCcw1MotorPoewrLevel(unsigned short CCW1);
void setupCcw2MotorPoewrLevel(unsigned short CCW2);
void setupCw1MotorPoewrLevel(unsigned short CW1);
void setupCw2MotorPoewrLevel(unsigned short CW2);
void setThrottlePowerLevel(unsigned short level);
unsigned short getThrottlePowerLevel();
unsigned short getMinPowerLevel();
unsigned short getMaxPowerLeve();
unsigned short getAdjustPowerLeveRange();
void setAdjustPowerLeveRange(int v);
unsigned short getPidOutputLimitation();
void setPidOutputLimitation(int v);
float getMotorGain(unsigned char index);
void setMotorGain(unsigned char index, float value);

