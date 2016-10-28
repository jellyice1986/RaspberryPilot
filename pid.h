
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
	float err;//current error
	struct timeval last_tv;
	float last_error; //last error  of pid calculation
}PID_STRUCT;

extern PID_STRUCT rollAttitudePidSettings;
extern PID_STRUCT pitchAttitudePidSettings;
extern PID_STRUCT yawAttitudePidSettings;
extern PID_STRUCT rollRatePidSettings;
extern PID_STRUCT pitchRatePidSettings;
extern PID_STRUCT yawRatePidSettings;
extern PID_STRUCT verticalHeightSettings;
extern PID_STRUCT verticalSpeedSettings;

void pidInit(void);
float pidCalculation(PID_STRUCT *pid, float process_point,const bool updateError);
void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain,
		float set_point, float shift, float ilimit);
void resetPidRecord(PID_STRUCT *pid);
void setPidError(PID_STRUCT *pi, float value);
float getPidSperror(PID_STRUCT *pi) ;
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



