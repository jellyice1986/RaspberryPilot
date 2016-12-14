/******************************************************************************
The pid.c in RaspberryPilot project is placed under the MIT license

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
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include "commonLib.h"
#include "pid.h"

/**
 *	Default PID parameter for attitude
 */
#define DEFAULT_ROLL_ATTITUDE_P_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_I_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_D_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_I_LIMIT 0.0

#define DEFAULT_PITCH_ATTITUDE_P_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_I_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_D_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_I_LIMIT 0.0

#define DEFAULT_YAW_ATTITUDE_P_GAIN 0.0
#define DEFAULT_YAW_ATTITUDE_I_GAIN 0.0
#define DEFAULT_YAW_ATTITUDE_D_GAIN 0.0
#define DEFAULT_YAW_ATTITUDE_I_LIMIT 0.0

#define DEFAULT_ROLL_ATTITUDE_SP 0.0
#define DEFAULT_PITCH_ATTITUDE_SP 0.0
#define DEFAULT_YAW_ATTITUDE_SP 0.0
#define DEFAULT_ROLL_ATTITUDE_SHIFT 0.0
#define DEFAULT_PITCH_ATTITUDE_SHIFT  0.0
#define DEFAULT_YAW_ATTITUDE_SHIFT 0.0
#define DEFAULT_ROLL_ATTITUDE_DEADBAND 0.05
#define DEFAULT_PITCH_ATTITUDE_DEADBAND 0.05
#define DEFAULT_YAW_ATTITUDE_DEADBAND 0.05

PID_STRUCT rollAttitudePidSettings;
PID_STRUCT pitchAttitudePidSettings;
PID_STRUCT yawAttitudePidSettings;

/**
 *	Default PID parameter for rate
 */
#define DEFAULT_ROLL_RATE_P_GAIN 0.0
#define DEFAULT_ROLL_RATE_I_GAIN 0.0
#define DEFAULT_ROLL_RATE_D_GAIN 0.0
#define DEFAULT_ROLL_RATE_I_LIMIT 0.0

#define DEFAULT_PITCH_RATE_P_GAIN 0.0
#define DEFAULT_PITCH_RATE_I_GAIN 0.0
#define DEFAULT_PITCH_RATE_D_GAIN 0.0
#define DEFAULT_PITCH_RATE_I_LIMIT 0.0

#define DEFAULT_YAW_RATE_P_GAIN 0.0
#define DEFAULT_YAW_RATE_I_GAIN 0.0
#define DEFAULT_YAW_RATE_D_GAIN 0.0
#define DEFAULT_YAW_RATE_I_LIMIT 0.0

#define DEFAULT_ROLL_RATE_SP 0.0
#define DEFAULT_PITCH_RATE_SP 0.0
#define DEFAULT_YAW_RATE_SP 0.0
#define DEFAULT_ROLL_RATE_SHIFT 0.0
#define DEFAULT_PITCH_RATE_SHIFT 0.0
#define DEFAULT_YAW_RATE_SHIFT 0.0
#define DEFAULT_ROLL_RATE_DEADBAND 1.0
#define DEFAULT_PITCH_RATE_DEADBAND 1.0
#define DEFAULT_YAW_RATE_DEADBAND 1.0

PID_STRUCT rollRatePidSettings;
PID_STRUCT pitchRatePidSettings;
PID_STRUCT yawRatePidSettings;

/**
 * Default PID parameter for AltHold altitude
 */
#define DEFAULT_ALTHOLD_ALT_P_GAIN 0.0
#define DEFAULT_ALTHOLD_ALT_I_GAIN 0.0
#define DEFAULT_ALTHOLD_ALT_D_GAIN 0.0
#define DEFAULT_ALTHOLD_ALT_I_LIMIT 0.0
#define DEFAULT_ALTHOLD_ALT_SP 0.0
#define DEFAULT_ALTHOLD_ALT_SHIFT 0.0
#define DEFAULT_ALTHOLD_ALT_DEADBAND 0.0

PID_STRUCT altHoldAltSettings;

/**
 * Default PID parameter for AltHold vertical speed
 */
#define DEFAULT_ALTHOLD_SPEED_P_GAIN 0.0
#define DEFAULT_ALTHOLD_SPEED_I_GAIN 0.0
#define DEFAULT_ALTHOLD_SPEED_D_GAIN 0.0
#define DEFAULT_ALTHOLD_SPEED_I_LIMIT 0.0
#define DEFAULT_ALTHOLD_SPEED_SP 0.0
#define DEFAULT_ALTHOLD_SPEED_SHIFT  0.0
#define DEFAULT_ALTHOLD_SPEED_DEADBAND 0.0

PID_STRUCT altHoldlSpeedSettings;

/**
 *  Init  PID controler
 *
 * @param
 * 		void
 *
 * @return
 *		 void
 *
 */
void pidInit() {

	//init PID controler for attitude	
	pidTune(&rollAttitudePidSettings, DEFAULT_ROLL_ATTITUDE_P_GAIN,
	DEFAULT_ROLL_ATTITUDE_I_GAIN, DEFAULT_ROLL_ATTITUDE_D_GAIN,
	DEFAULT_ROLL_ATTITUDE_SP, DEFAULT_ROLL_ATTITUDE_SHIFT,
	DEFAULT_ROLL_ATTITUDE_I_LIMIT,DEFAULT_ROLL_ATTITUDE_DEADBAND);
	pidTune(&pitchAttitudePidSettings, DEFAULT_PITCH_ATTITUDE_P_GAIN,
	DEFAULT_PITCH_ATTITUDE_I_GAIN, DEFAULT_PITCH_ATTITUDE_D_GAIN,
	DEFAULT_PITCH_ATTITUDE_SP, DEFAULT_PITCH_ATTITUDE_SHIFT,
	DEFAULT_PITCH_ATTITUDE_I_LIMIT,DEFAULT_PITCH_ATTITUDE_DEADBAND);
	pidTune(&yawAttitudePidSettings, DEFAULT_YAW_ATTITUDE_P_GAIN,
	DEFAULT_YAW_ATTITUDE_I_GAIN, DEFAULT_YAW_ATTITUDE_D_GAIN,
	DEFAULT_YAW_ATTITUDE_SP, DEFAULT_YAW_ATTITUDE_SHIFT,
	DEFAULT_YAW_ATTITUDE_I_LIMIT,DEFAULT_YAW_ATTITUDE_DEADBAND);
	setName(&rollAttitudePidSettings, "ROLL_A");
	setName(&pitchAttitudePidSettings, "PITCH_A");
	setName(&yawAttitudePidSettings, "YAW_A");
	resetPidRecord(&rollAttitudePidSettings);
	resetPidRecord(&pitchAttitudePidSettings);
	resetPidRecord(&yawAttitudePidSettings);
	setPidSp(&yawAttitudePidSettings, 321.0);

	//init PID controler for rate
	pidTune(&rollRatePidSettings, DEFAULT_ROLL_RATE_P_GAIN,
	DEFAULT_ROLL_RATE_I_GAIN, DEFAULT_ROLL_RATE_D_GAIN,
	DEFAULT_ROLL_RATE_SP, DEFAULT_ROLL_RATE_SHIFT,
	DEFAULT_ROLL_RATE_I_LIMIT,DEFAULT_ROLL_RATE_DEADBAND);
	pidTune(&pitchRatePidSettings, DEFAULT_PITCH_RATE_P_GAIN,
	DEFAULT_PITCH_RATE_I_GAIN, DEFAULT_PITCH_RATE_D_GAIN,
	DEFAULT_PITCH_RATE_SP, DEFAULT_PITCH_RATE_SHIFT,
	DEFAULT_PITCH_RATE_I_LIMIT,DEFAULT_PITCH_RATE_DEADBAND);
	pidTune(&yawRatePidSettings, DEFAULT_YAW_RATE_P_GAIN,
	DEFAULT_YAW_RATE_I_GAIN, DEFAULT_YAW_RATE_D_GAIN,
	DEFAULT_YAW_RATE_SP, DEFAULT_YAW_RATE_SHIFT,
	DEFAULT_YAW_RATE_I_LIMIT,DEFAULT_YAW_RATE_DEADBAND);
	setName(&rollRatePidSettings, "ROLL_R");
	setName(&pitchRatePidSettings, "PITCH_R");
	setName(&yawRatePidSettings, "YAW_R");
	resetPidRecord(&rollRatePidSettings);
	resetPidRecord(&pitchRatePidSettings);
	resetPidRecord(&yawRatePidSettings);

	//init PID controler for vertical height
	pidTune(&altHoldAltSettings, DEFAULT_ALTHOLD_ALT_P_GAIN,
	DEFAULT_ALTHOLD_ALT_I_GAIN, DEFAULT_ALTHOLD_ALT_D_GAIN,
	DEFAULT_ALTHOLD_ALT_SP, DEFAULT_ALTHOLD_ALT_SHIFT,
	DEFAULT_ALTHOLD_ALT_I_LIMIT,DEFAULT_ALTHOLD_ALT_DEADBAND);
	setName(&altHoldAltSettings, "VH");
	resetPidRecord(&altHoldAltSettings);

	//init PID controler for vertical speed
	pidTune(&altHoldlSpeedSettings, DEFAULT_ALTHOLD_SPEED_P_GAIN,
	DEFAULT_ALTHOLD_SPEED_I_GAIN, DEFAULT_ALTHOLD_SPEED_D_GAIN,
	DEFAULT_ALTHOLD_SPEED_SP, DEFAULT_ALTHOLD_SPEED_SHIFT,
	DEFAULT_ALTHOLD_SPEED_I_LIMIT,DEFAULT_ALTHOLD_SPEED_DEADBAND);
	setName(&altHoldlSpeedSettings, "VS");
	resetPidRecord(&altHoldlSpeedSettings);
}

/**
 * PID conrroler
 *
 * @param pid
 *		 pid entity
 *
 * @param processValue
 *		input of PID controler
 *
 * @return
 *		output of PID controler
 *
 */
float pidCalculation(PID_STRUCT *pid, float processValue) {

	float pterm = 0.f;
	float dterm = 0.f;
	float iterm = 0.f;
	float result = 0.f;
	float timeDiff = 0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (pid->last_tv.tv_sec != 0) {

		pid->pv = processValue;
		timeDiff = ((float) (tv.tv_sec - pid->last_tv.tv_sec)
				+ (float)(tv.tv_usec - pid->last_tv.tv_usec)*0.000001f);

		//P term
		pid->err = deadband((pid->sp + pid->spShift) - (pid->pv), pid->deadBand) ;

		//I term
		pid->integral += (pid->err * timeDiff);
		pid->integral=LIMIT_MIN_MAX_VALUE(pid->integral,-pid->iLimit, pid->iLimit);

		//D term
		dterm = (pid->err - pid->last_error) / timeDiff;

		pterm = pid->pgain * pid->err;
		iterm = pid->igain * pid->integral;
		dterm = pid->dgain * dterm;

		result = (pterm + iterm + dterm);

#if 0 //Debug
		if(0==strncmp(pid->name,"ROLL_R",strlen("ROLL_R"))){
			_DEBUG(DEBUG_NORMAL,"name       =%s\n"	,getName(pid));
			_DEBUG(DEBUG_NORMAL,"timeDiff   =%.3f\n",timeDiff);
			_DEBUG(DEBUG_NORMAL,"result     =%.3f\n",result);
			_DEBUG(DEBUG_NORMAL,"pterm      =%.3f\n",pterm);
			_DEBUG(DEBUG_NORMAL,"iterm      =%.3f\n",iterm);
			_DEBUG(DEBUG_NORMAL,"iintegral  =%.3f\n",pid->integral);
			_DEBUG(DEBUG_NORMAL,"dterm      =%.3f\n",dterm);
			_DEBUG(DEBUG_NORMAL,"sp         =%.3f\n",pid->sp);
			_DEBUG(DEBUG_NORMAL,"spshift    =%.3f\n", pid->spShift);
			_DEBUG(DEBUG_NORMAL,"pv		    =%.3f\n",pid->pv);
			_DEBUG(DEBUG_NORMAL,"err        =%.3f\n",pid->err);
			_DEBUG(DEBUG_NORMAL,"last_error =%.3f\n",pid->last_error);
		}
#endif
	}

	pid->last_error = pid->err;
	pid->last_tv.tv_sec = tv.tv_sec;
	pid->last_tv.tv_usec = tv.tv_usec;

	return result;
}

/**
 * tune PID conrroler
 *
 * @param pid
 *		 pid entity
 *
 * @param p_gain
 *		 P item
 *
 * @param i_gain
 *		 I item
 *
 * @param d_gain
 *		 D item
 *
 * @param set_point
 *		 set point
 *
 * @param shift
 *		 a shift for set point
 *
 * @param iLimit
 *		limition for output of I item
 *
 * @param deadBand
 *		dead band
 *
 * @return
 *		output of PID controler
 *
 */
void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain,
		float set_point, float shift, float iLimit,float deadBand) {

	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->iLimit = iLimit;
	pid->dgain = d_gain;
	pid->sp = set_point;
	pid->spShift = shift;
	pid->deadBand=deadBand;
}

/**
 *  reset PID record
 *
 * @param pid
 * 		PID record
 *
 * @return
 *		 void
 *
 */
void resetPidRecord(PID_STRUCT *pid) {
	pid->integral = 0.f;
	pid->err = 0.f;
	pid->last_error = 0.f;
	pid->last_tv.tv_usec = 0;
	pid->last_tv.tv_sec = 0;
}

/**
 *  set error to PID controler
 *
 * @param pi
 * 		PID entity
 *
 * @param value
 * 		error for PID controler
 *
 * @return
 *		 void
 *
 */
void setPidError(PID_STRUCT *pi, float value) {
	pi->err = value;
}

/**
 *  get error from PID record
 *
 * @param pi
 * 		PID entity
 *
 * @return
 *		 error
 *
 */
float getPidSperror(PID_STRUCT *pi) {
	return pi->err;
}

/**
 *  set a shift for set point
 *
 * @param pi
 * 		PID entity
 *
 * @param value
 * 		value
 *
 * @return
 *		 void
 *
 */
void setPidSpShift(PID_STRUCT *pi, float value) {
	pi->spShift = value;
}

/**
 *  get  shift
 *
 * @param pi
 * 		PID entity
 *
 * @return
 *		 shift
 *
 */
float getPidSpShift(PID_STRUCT *pi) {
	return pi->spShift;
}

/**
 *  set set point
 *
 * @param pid
 * 		PID entity
 *
 * @param set_point
 * 		set point
 *
 * @return
 *		 void
 *
 */
void setPidSp(PID_STRUCT *pid, float set_point) {
	pid->sp = set_point;
}

/**
 *  get set point
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 set point
 *
 */
float getPidSp(PID_STRUCT *pid) {
	return pid->sp;
}

/**
 *  name a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param name
 * 		name
 *
 * @return
 *		 void
 *
 */
void setName(PID_STRUCT *pid, char *name) {
	strcpy(pid->name, name);
}

/**
 *  get the name of a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 name
 *
 */
char *getName(PID_STRUCT *pid) {
	return pid->name;
}

/**
 *  set P for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		P item
 *
 * @return
 *		 void
 *
 */
void setPGain(PID_STRUCT *pid, float gain) {
	pid->pgain = gain;
}

/**
 *  get P from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 P item
 *
 */
float getPGain(PID_STRUCT *pid) {
	return pid->pgain;
}

/**
 *  set I for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		I item
 *
 * @return
 *		 void
 *
 */
void setIGain(PID_STRUCT *pid, float gain) {
	pid->igain = gain;
}

/**
 *  get I from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 I item
 *
 */
float getIGain(PID_STRUCT *pid) {
	return pid->igain;
}

/**
 *  set I limitation for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		I limitation
 *
 * @return
 *		 void
 *
 */
void setILimit(PID_STRUCT *pid, float v) {
	pid->iLimit = v;
}

/**
 *  get limitation of I from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		limitation of I item
 *
 */
float getILimit(PID_STRUCT *pid) {
	return pid->iLimit;
}

/**
 *  set D for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		D item
 *
 * @return
 *		 void
 *
 */
void setDGain(PID_STRUCT *pid, float gain) {
	pid->dgain = gain;
}

/**
 *  get D from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 D item
 *
 */
float getDGain(PID_STRUCT *pid) {
	return pid->dgain;
}

