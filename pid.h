/******************************************************************************
The pid.h in RaspberryPilot project is placed under the MIT license

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

typedef struct {
	char name[10]; //name of pid entity
	float pv; //process value
	float sp; //set point, it is usually changed by remote controller
	float spShift; //sp shift, it is usually used to calibrate zero point
	float integral; // accumulated error
	float pgain; //Kp 
	float igain; //Ki
	float iLimit; //limitation of integral
	float dgain; //Kd
	float err; //current error
	float deadBand;
	struct timeval last_tv;
	float last_error; //last error  of pid calculation
} PID_STRUCT;

extern PID_STRUCT rollAttitudePidSettings;
extern PID_STRUCT pitchAttitudePidSettings;
extern PID_STRUCT yawAttitudePidSettings;
extern PID_STRUCT rollRatePidSettings;
extern PID_STRUCT pitchRatePidSettings;
extern PID_STRUCT yawRatePidSettings;
extern PID_STRUCT altHoldAltSettings;
extern PID_STRUCT altHoldlSpeedSettings;

void pidInit(void);
float pidCalculation(PID_STRUCT *pid, float process_point);
void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain,
		float set_point, float shift, float ilimit,float deadBand);
void resetPidRecord(PID_STRUCT *pid);
void setPidError(PID_STRUCT *pi, float value);
float getPidSperror(PID_STRUCT *pi);
void setPidSp(PID_STRUCT *pid, float set_point);
float getPidSp(PID_STRUCT *pid);
void setPidSpShift(PID_STRUCT *pi, float value);
float getPidSpShift(PID_STRUCT *pi);
void setName(PID_STRUCT *pid, char *name);
char *getName(PID_STRUCT *pid);
void setPGain(PID_STRUCT *pid, float gain);
float getPGain(PID_STRUCT *pid);
void setIGain(PID_STRUCT *pid, float gain);
float getIGain(PID_STRUCT *pid);
void setILimit(PID_STRUCT *pid, float v);
float getILimit(PID_STRUCT *pid);
void setDGain(PID_STRUCT *pid, float gain);
float getDGain(PID_STRUCT *pid);

