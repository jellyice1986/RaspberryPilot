#include <stdio.h>
#include <math.h>
#include <string.h>

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

PID_STRUCT rollRatePidSettings;
PID_STRUCT pitchRatePidSettings;
PID_STRUCT yawRatePidSettings;

/**
 * Default PID parameter for vertical height
 */
#define DEFAULT_VERTICAL_HEIGHT_P_GAIN 0.0
#define DEFAULT_VERTICAL_HEIGHT_I_GAIN 0.0
#define DEFAULT_VERTICAL_HEIGHT_D_GAIN 0.0
#define DEFAULT_VERTICAL_HEIGHT_I_LIMIT 0.0
#define DEFAULT_VERTICAL_HEIGHT_SP 0.0
#define DEFAULT_VERTICAL_HEIGHT_SHIFT 0.0
PID_STRUCT verticalHeightSettings;

/**
 * Default PID parameter for vertical speed
 */
#define DEFAULT_VERTICAL_SPEED_P_GAIN 0.0
#define DEFAULT_VERTICAL_SPEED_I_GAIN 0.0
#define DEFAULT_VERTICAL_SPEED_D_GAIN 0.0
#define DEFAULT_VERTICAL_SPEED_I_LIMIT 0.0
#define DEFAULT_VERTICAL_SPEED_SP 0.0
#define DEFAULT_VERTICAL_SPEED_SHIFT  0.0

PID_STRUCT verticalSpeedSettings;

#ifdef MPU_DMP
#define PID_SAMPLE_TIME 0.02   // 20 mSec
#else
#define PID_SAMPLE_TIME 0.005  // 5 mSec
#endif

/**
 * Init paramtes and states of PID controler
 */
void pidInit() {

	//init PID controler for attitude
#ifdef MPU_DMP
	//adjustment period=0.02 Sec;
	//adjustment freq= 50 HZ
#else
	//adjustment period=0.005 Sec;
	//adjustment freq= 200 HZ
#endif	
	pidTune(&rollAttitudePidSettings, DEFAULT_ROLL_ATTITUDE_P_GAIN,
	DEFAULT_ROLL_ATTITUDE_I_GAIN, DEFAULT_ROLL_ATTITUDE_D_GAIN,
	DEFAULT_ROLL_ATTITUDE_SP, DEFAULT_ROLL_ATTITUDE_SHIFT,
	DEFAULT_ROLL_ATTITUDE_I_LIMIT, PID_SAMPLE_TIME);
	pidTune(&pitchAttitudePidSettings, DEFAULT_PITCH_ATTITUDE_P_GAIN,
	DEFAULT_PITCH_ATTITUDE_I_GAIN, DEFAULT_PITCH_ATTITUDE_D_GAIN,
	DEFAULT_PITCH_ATTITUDE_SP, DEFAULT_PITCH_ATTITUDE_SHIFT,
	DEFAULT_PITCH_ATTITUDE_I_LIMIT, PID_SAMPLE_TIME);
	pidTune(&yawAttitudePidSettings, DEFAULT_YAW_ATTITUDE_P_GAIN,
	DEFAULT_YAW_ATTITUDE_I_GAIN, DEFAULT_YAW_ATTITUDE_D_GAIN,
	DEFAULT_YAW_ATTITUDE_SP, DEFAULT_YAW_ATTITUDE_SHIFT,
	DEFAULT_YAW_ATTITUDE_I_LIMIT, PID_SAMPLE_TIME);
	setName(&rollAttitudePidSettings, "ROLL_A");
	setName(&pitchAttitudePidSettings, "PITCH_A");
	setName(&yawAttitudePidSettings, "YAW_A");
	resetPidRecord(&rollAttitudePidSettings);
	resetPidRecord(&pitchAttitudePidSettings);
	resetPidRecord(&yawAttitudePidSettings);
	setPidSp(&yawAttitudePidSettings, 321.0); //321 is a  value for yaw while while throttle is minimum

	//init PID controler for rate
	pidTune(&rollRatePidSettings, DEFAULT_ROLL_RATE_P_GAIN,
	DEFAULT_ROLL_RATE_I_GAIN, DEFAULT_ROLL_RATE_D_GAIN,
	DEFAULT_ROLL_RATE_SP, DEFAULT_ROLL_RATE_SHIFT,
	DEFAULT_ROLL_RATE_I_LIMIT, PID_SAMPLE_TIME);
	pidTune(&pitchRatePidSettings, DEFAULT_PITCH_RATE_P_GAIN,
	DEFAULT_PITCH_RATE_I_GAIN, DEFAULT_PITCH_RATE_D_GAIN,
	DEFAULT_PITCH_RATE_SP, DEFAULT_PITCH_RATE_SHIFT,
	DEFAULT_PITCH_RATE_I_LIMIT, PID_SAMPLE_TIME);
	pidTune(&yawRatePidSettings, DEFAULT_YAW_RATE_P_GAIN,
	DEFAULT_YAW_RATE_I_GAIN, DEFAULT_YAW_RATE_D_GAIN,
	DEFAULT_YAW_RATE_SP, DEFAULT_YAW_RATE_SHIFT,
	DEFAULT_YAW_RATE_I_LIMIT, PID_SAMPLE_TIME);
	setName(&rollRatePidSettings, "ROLL_R");
	setName(&pitchRatePidSettings, "PITCH_R");
	setName(&yawRatePidSettings, "YAW_R");
	resetPidRecord(&rollRatePidSettings);
	resetPidRecord(&pitchRatePidSettings);
	resetPidRecord(&yawRatePidSettings);

	//init PID controler for vertical height
	pidTune(&verticalHeightSettings, DEFAULT_VERTICAL_HEIGHT_P_GAIN,
	DEFAULT_VERTICAL_HEIGHT_I_GAIN, DEFAULT_VERTICAL_HEIGHT_D_GAIN,
	DEFAULT_VERTICAL_HEIGHT_SP, DEFAULT_VERTICAL_HEIGHT_SHIFT,
	DEFAULT_VERTICAL_HEIGHT_I_LIMIT, PID_SAMPLE_TIME);
	setName(&verticalHeightSettings, "VH");
	resetPidRecord(&verticalHeightSettings);

	//init PID controler for vertical speed
	pidTune(&verticalSpeedSettings, DEFAULT_VERTICAL_SPEED_P_GAIN,
	DEFAULT_VERTICAL_SPEED_I_GAIN, DEFAULT_VERTICAL_SPEED_D_GAIN,
	DEFAULT_VERTICAL_SPEED_SP, DEFAULT_VERTICAL_SPEED_SHIFT,
	DEFAULT_VERTICAL_SPEED_I_LIMIT, PID_SAMPLE_TIME);
	setName(&verticalSpeedSettings, "VS");
	resetPidRecord(&verticalSpeedSettings);

}

/**
 * Execute PID calculation by a input  value
 * @param pid
 *		structure of pid information
 *
 * @param processValue
 *		input value for PID calculation
 *
 * @return
 *		output of PID calculation
 *
 */
 
float pidCalculation(PID_STRUCT *pid, float processValue,
		const bool updateError) {

	float pterm = 0, dterm = 0, iterm = 0, result = 0, timeDiff=0;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (pid->last_tv.tv_sec != 0) {

		pid->pv = processValue;
		timeDiff=(float)((float)(tv.tv_sec-pid->last_tv.tv_sec)+(float)(tv.tv_usec-pid->last_tv.tv_usec)/1000000.f);

		//P term
		if (updateError) {
			pid->err = (pid->sp + pid->spShift) - (pid->pv);
		}

		//I term
		pid->integral += (pid->err * timeDiff);
		if (pid->integral > pid->iLimit) {
			pid->integral = pid->iLimit;
		} else if (pid->integral < -pid->iLimit) {
			pid->integral = -pid->iLimit;
		}

		//D term
		dterm = (pid->err - pid->last_error) / timeDiff;

		pterm = pid->pgain * pid->err;
		iterm = pid->igain * pid->integral;
		dterm = pid->dgain * dterm;

		result = (pterm + iterm + dterm);

#if 0
		//if(pid->name[0]=='R')
		//printf("name=%s, result=%3.3f, pterm=%2.5f, iterm=%2.5f, pid->integral=%3.5f, dterm=%2.5f, sp=%3.2f, shift=%3.2f,pid->last_error=%3.2f,p=%3.5f,d=%3.5f\n",
		//		getName(pid),result,pterm,iterm,pid->integral,dterm,pid->sp,pid->spShift,pid->last_error,pid->pgain,pid->dgain);
		if(pid->name[0]=='P')
		printf("name=%s, timeDiff=%.3f, result=%.3f, pterm=%.3f, iterm=%.3f, iintegral=%.3f, dterm=%.3f, sp=%.3f, shift=%.3f, pv=%.3f, err=%.3f, last_error=%.3f\n",
				getName(pid),timeDiff,result,pterm,iterm,pid->integral,dterm,pid->sp,pid->spShift,pid->pv,pid->err,pid->last_error);
#endif
	}
	
	pid->last_error = pid->err;
	pid->last_tv.tv_sec = tv.tv_sec;
	pid->last_tv.tv_usec = tv.tv_usec;

	return result;
}

/**
 * Tune PID parameter
 */
void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain,
		float set_point, float shift, float iLimit, float dt) {
	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->dgain = d_gain;
	pid->sp = set_point;
	pid->spShift = shift;
	pid->iLimit = iLimit;
	pid->dt = dt;
}

/**
 * reset recarded of PID controler
 */
void resetPidRecord(PID_STRUCT *pid) {
	pid->integral = 0.f;
	pid->err = 0.f;
	pid->last_error = 0.f;
	pid->last_tv.tv_usec = 0;
	pid->last_tv.tv_sec = 0;
}

void setPidError(PID_STRUCT *pi, float value) {
	pi->err = value;
}

float getPidSperror(PID_STRUCT *pi) {
	return pi->err;
}

/**
 * set SP shift, SP shift is usually used to calibrate zero point
 */
void setPidSpShift(PID_STRUCT *pi, float value) {
	pi->spShift = value;
}

/**
 * get SP shift, SP shift is usually used to calibrate zero point
 */
float getPidSpShift(PID_STRUCT *pi) {
	return pi->spShift;
}

/**
 * set SP, SP is usually changed by remote controler.
 */
void setPidSp(PID_STRUCT *pid, float set_point) {
	pid->sp = set_point;
}

/**
 * get SP, SP is usually changed by remote controler.
 */
float getPidSp(PID_STRUCT *pid) {
	return pid->sp;
}

/**
 * give a PID structure a name to, this name is usually used to debug.
 */
void setName(PID_STRUCT *pid, char *name) {
	strcpy(pid->name, name);
}

/**
 * get the name of a PID structure, this name is usually used to debug.
 */
char *getName(PID_STRUCT *pid) {
	return pid->name;
}

/**
 * set kp
 */
void setPGain(PID_STRUCT *pid, float gain) {
	pid->pgain = gain;
}

/**
 * get kp
 */
float getPGain(PID_STRUCT *pid) {
	return pid->pgain;
}

/**
 * set ki
 */
void setIGain(PID_STRUCT *pid, float gain) {
	pid->igain = gain;
}

/**
 * set ki
 */
float getIGain(PID_STRUCT *pid) {
	return pid->igain;
}

/**
 * set the limitation of integration
 */
void setILimit(PID_STRUCT *pid, float v) {
	pid->iLimit = v;
}

/**
 * get the limitation of integration
 */
float getILimit(PID_STRUCT *pid) {
	return pid->iLimit;
}

/**
 * set kd
 */
void setDGain(PID_STRUCT *pid, float gain) {
	pid->dgain = gain;
}

/**
 * get kd
 */
float getDGain(PID_STRUCT *pid) {
	return pid->dgain;
}

