

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <wiringSerial.h>
#include "commonLib.h"
#include "flyControler.h"
#include "pid.h"
#include "motorControl.h"
#include "systemControl.h"
#include "mpu6050.h"
#include "radioControl.h"
#include "securityMechanism.h"

static int serialFd;
static pthread_t radioThreadId;
static pthread_t transmitThreadId;;

void *radioReceiveThread(void *arg);
void *radioTransmitThread(void *arg);
short processRadioMessages(int fd, char *buf, short lenth);
bool extractPacketInfo(char *buf,int lenth, char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]);

#define CHECK_RECEIVER_PERIOD 0

/**
* init paramtes and states for radio
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
bool radioControlInit(){

	serialFd=0;

	if ((serialFd = serialOpen("/dev/ttyAMA0", SERIAL_BAUDRATE)) < 0) {
		_DEBUG(DEBUG_NORMAL,"Unable to open serial device: %s\n", strerror(errno));
		return false;
	}
	
	serialFlush (serialFd);

	if (pthread_create(&transmitThreadId, NULL, radioTransmitThread, &serialFd)) {
		_DEBUG(DEBUG_NORMAL,"transmitThreadId create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL,"start transmiting...\n");
	}

	//start radio receiver thread
	if (pthread_create(&radioThreadId, NULL, radioReceiveThread, &serialFd)) {
		_DEBUG(DEBUG_NORMAL,"radioThreadId create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL,"start reveiveing...\n");
	}
	return true;
}


/**
*  transmit packets to remot controler
*
* @param arg
* 		file descriptor of tty of raspberry pi  
*
* @return 
*		void
*
*/
void *radioTransmitThread(void *arg) {

	char message[150];
	int fd = *(int *) arg;
	
	while (!getLeaveFlyControlerFlag()) {

		//    2 CCW2  CW2 3
		//          X
		//    1 CW1  CCW1 0
		//          H

		/** packet format
		@
		 roll attitude:
		 pitch attitude:
		 yaw sttitude:
		 rollAttitudeSp:
		 pitchAttitudeSp:
		 yawAttitudeSp:
		 roll gyro:
		 pitch gyro:
		 yaw gyro:
		 center throttle:
		 ccw1 throttle:
		 cw1 throttle :
		 ccw2 throttle:
		 cw2 throttle:
		 #
		 */
		snprintf(message,sizeof(message),
				"@%.1f:%.1f:%.1f:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
				getRoll(), getPitch(), getYaw(), (int)getPidSp(&rollAttitudePidSettings),
				(int)getPidSp(&pitchAttitudePidSettings),  (int)(getYawCenterPoint()+getPidSp(&yawAttitudePidSettings)),
				(int)getRollGyro(), (int)getPitchGyro(), (int)getYawGyro(),
				getThrottlePowerLevel(), getMotorPowerLevelCCW1(),
				getMotorPowerLevelCW1(), getMotorPowerLevelCCW2(),
				getMotorPowerLevelCW2());
		
		if ('#'!=message[strlen(message)-1]){
			_DEBUG(DEBUG_NORMAL,("buffer overflow\n"));
		}else{
			serialPuts(fd, message);
			increasePacketCounter();
			memset(message, '\0', sizeof(message));
		}
		
		usleep(TRANSMIT_TIMER);
	}

	pthread_exit((void *)1234);
}

/**
*  reveive packets from remot controler
*
* @param arg
* 		file descriptor of tty of raspberry pi  
*
* @return 
*		void
*
*/
void *radioReceiveThread(void *arg) {

	int fd = *(int *) arg;
	char buf[300];
	char getChar;
	unsigned char count = 0;

	memset(buf, '\0', sizeof(buf));

	while(!getLeaveFlyControlerFlag()) {

		if (serialDataAvail(fd)) {

			getChar = serialGetchar(fd);
			
			if (getChar == '@') {
				memset(buf, '\0', sizeof(buf));
				buf[0] = getChar;
				count = 1;
			} else if ((getChar == '#') && (buf[0] == '@')
					&& (count < sizeof(buf))) {
				buf[count] = getChar;
				processRadioMessages(fd, buf, sizeof(buf));
				memset(buf, '\0', sizeof(buf));
				count = 0;
			} else {
				if (buf[0] == '@' && (count < sizeof(buf))) {
					buf[count] = getChar;
					count++;
				} else {
					memset(buf, '\0', sizeof(buf));
					count = 0;
				}

			}
		}
		usleep(500);
	}

	pthread_exit((void *)1234);
}


/**
*  extract info from a received packet
*
* @param buf
* 		packet  
*
* @param lenth
* 		lentg of packet  
*
* @param container
* 		packet  info container
*
* @return 
*		bool
*
*/
bool extractPacketInfo(char *buf,int lenth, char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]){

	int i=0;
	char *token=NULL;
	char cmd[lenth];
	
	for (i = 1; (buf[i] != '#')&&(i<lenth); i++) {
		cmd[i - 1] = buf[i];
	}
	cmd[i-1]='\0';
	i=1;
	token = strtok(cmd, ":");
	strncpy(container[0],token,strlen(token));

	if(token==NULL) return false;
		
	do{
		token = strtok(NULL, ":");
		if(token==NULL){
			break; 
		}else{
			strncpy(container[i],token,strlen(token));
			i++;
		}
	}while(true);
	
	return true;
		
}

/**
 * Decode a received packet
 *
 * @param buf
 * 		received packet
 *
 * @param lenth
 * 		packet size
 */
short processRadioMessages(int fd, char *buf, short lenth) {

	char packet[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH];
	short parameter = 0;
	float parameterF = 0.0;
	float rollSpShift = 0;
	float pitchSpShift = 0;
	float yawShiftValue=0;
	float throttlePercentage=0;
#if CHECK_RECEIVER_PERIOD	
	struct timeval tv;
	struct timeval tv_last;
#endif

	resetPacketCounter();

	if(!extractPacketInfo(buf,lenth,packet)) return false;


	switch (atoi(packet[0])) {
		
		case HEADER_ENABLE_FLY_SYSTEM:
		// Enable or disable fly syatem
		
		if (1 == atoi(packet[ENABLE_FLY_SYSTEM_FIWLD_ISENABLE])) {
			if (!flySystemIsEnable()) {
				enableFlySystem();
				motorInit();
			}
		} else {
			disenableFlySystem();
		}

		break;
		
		case HEADER_CONTROL_MOTION:
		//control packet
		
#if CHECK_RECEIVER_PERIOD
				gettimeofday(&tv,NULL);
				_DEBUG("duration=%d us\n",(tv.tv_sec-tv_last.tv_sec)*1000000+(tv.tv_usec-tv_last.tv_usec));
				tv_last.tv_usec=tv.tv_usec;
				tv_last.tv_sec=tv.tv_sec;
#endif
		
		rollSpShift = atof(packet[CONTROL_MOTION_ROLL_SP_SHIFT]);
		pitchSpShift = atof(packet[CONTROL_MOTION_PITCH_SP_SHIFT]);
		yawShiftValue = atof(packet[CONTROL_MOTION_YAW_SHIFT_VALUE]);
		throttlePercentage=atof(packet[CONTROL_MOTION_THROTTLE]) / 100.f;
		parameter = getMinPowerLevel()
				+ (int) (throttlePercentage
						* (float) (getMaxPowerLeve()
								- getMinPowerLevel())); 
		
		if (parameter > getMaxPowerLeve()
				|| parameter < getMinPowerLevel()) {
			_DEBUG(DEBUG_NORMAL,"invilid throttle level\n");
			break;
		}
		
		if (true==flySystemIsEnable()) {
			
			pthread_mutex_lock(&controlMotorMutex);

			setThrottlePowerLevel(parameter);
			
			if (getMinPowerLevel() == parameter) {
				resetPidRecord(&rollAttitudePidSettings);
				resetPidRecord(&pitchAttitudePidSettings);
				resetPidRecord(&yawAttitudePidSettings);
				resetPidRecord(&rollRatePidSettings);
				resetPidRecord(&pitchRatePidSettings);
				setYawCenterPoint(0);
				setPidSp(&yawAttitudePidSettings, 321.0);

			} else {
				if (getPidSp(&yawAttitudePidSettings) == 321.0) {
					_DEBUG(DEBUG_NORMAL,"START Flying\n");
					setYawCenterPoint(getYaw());
					setPidSp(&yawAttitudePidSettings, 0);
				}
				setPidSp(&rollAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(rollSpShift, -getAngularLimit(),
								getAngularLimit()));
				setPidSp(&pitchAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(pitchSpShift, -getAngularLimit(),
								getAngularLimit()));
				setYawCenterPoint(getYawCenterPoint()+(yawShiftValue*1.0));
			}
			pthread_mutex_unlock(&controlMotorMutex);

		}
		break;

	case HEADER_HALT_PI:
		//halt raspberry pi
		
		pthread_mutex_lock(&controlMotorMutex);
		setupAllMotorPoewrLevel(getMinPowerLevel(), getMinPowerLevel(),
				getMinPowerLevel(), getMinPowerLevel());
		setLeaveFlyControlerFlag(true);
		pthread_mutex_unlock(&controlMotorMutex);
		system("sudo halt");
		break;

	case HEADER_SETUP_FACTOR:
		//setup factor
		
		_DEBUG(DEBUG_NORMAL,"%s %d: %s\n", __func__, __LINE__, buf);
		/***/
		parameter = atoi(packet[SETUP_FACTOR_PERIOD]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPeriod(parameter);
		_DEBUG(DEBUG_NORMAL,"Adjustment Period: %d\n", getAdjustPeriod());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LEVEL_RANGE]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPowerLeveRange(parameter);
		_DEBUG(DEBUG_NORMAL,"Adjustment Range: %d\n", getAdjustPowerLeveRange());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LIMIT]);
		if (parameter == 0) {
			parameter = 1;
		}
		setPidOutputLimitation(parameter);
		_DEBUG(DEBUG_NORMAL,"PID Output Limitation: %d\n", getPidOutputLimitation());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_GYRO_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setGyroLimit(parameterF);
		_DEBUG(DEBUG_NORMAL,"Angular Velocity Limit: %5.3f\n", getGyroLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ANGULAR_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setAngularLimit(parameterF);
		_DEBUG(DEBUG_NORMAL,"Roll/Pitch Angular Limit: %5.3f\n", getAngularLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ROLL_CAL]);
		setPidSpShift(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Roll Angular Calibration: %5.3f\n",
				getPidSpShift(&rollAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_PITCH_CAL]);
		setPidSpShift(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Pitch Angular Calibration: %5.3f\n",
				getPidSpShift(&pitchAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_0]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW1, parameterF);
		_DEBUG(DEBUG_NORMAL,"Motor 0 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_1]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW1, parameterF);
		_DEBUG(DEBUG_NORMAL,"Motor 1 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_2]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW2, parameterF);
		_DEBUG(DEBUG_NORMAL,"Motor 2 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW2));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_3]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW2, parameterF);
		_DEBUG(DEBUG_NORMAL,"Motor 3 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW2));
		/***/
		parameter = atoi(packet[SETUP_FACTOR_VERTICAL_HOLD_ENABLE]);
		/***/
		break;

	case HEADER_OlED_DISPLAY: 
		// oled control, doesn't exist anymore
		break;

	case HEADER_SETUP_PID:
		//setup pid parameters

		//attitude Roll P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_P]);
		setPGain(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Roll P Gain=%4.6f\n", getPGain(&rollAttitudePidSettings));

		//attitude Roll I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I]);
		setIGain(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Roll I Gain=%4.6f\n", getIGain(&rollAttitudePidSettings));

		//attitude Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I_LIMIT]);
		setILimit(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Roll I Output Limit=%4.6f\n",
				getILimit(&rollAttitudePidSettings));

		//attitude Roll D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_D]);
		setDGain(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Roll D Gain=%4.6f\n", getDGain(&rollAttitudePidSettings));

		//attitude Pitch P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_P]);
		setPGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Pitch P Gain=%4.6f\n", getPGain(&pitchAttitudePidSettings));

		//attitude Pitch I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I]);
		setIGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Pitch I Gain=%4.6f\n", getIGain(&pitchAttitudePidSettings));

		//attitude Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I_LIMIT]);
		setILimit(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchAttitudePidSettings));

		//attitude Pitch D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_D]);
		setDGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Pitch D Gain=%4.6f\n", getDGain(&pitchAttitudePidSettings));

		//attitude Yaw P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_P]);
		setPGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Yaw P Gain=%4.6f\n", getPGain(&yawAttitudePidSettings));

		//attitude Yaw I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I]);
		setIGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Yaw I Gain=%4.6f\n", getIGain(&yawAttitudePidSettings));

		//attitude Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I_LIMIT]);
		setILimit(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Yaw I Output Limit=%4.6f\n",
				getILimit(&yawAttitudePidSettings));

		//attitude Yaw D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_D]);
		setDGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Attitude Yaw D Gain=%4.6f\n", getDGain(&yawAttitudePidSettings));

		//Rate Roll P gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_P]);
		setPGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Roll P Gain=%4.6f\n", getPGain(&rollRatePidSettings));
		//Rate Roll I gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I]);
		setIGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Roll I Gain=%4.6f\n", getIGain(&rollRatePidSettings));
		//Rate Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I_LIMIT]);
		setILimit(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Roll I Output Limit=%4.6f\n",
				getILimit(&rollRatePidSettings));
		//Rate Roll D gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_D]);
		setDGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Roll D Gain=%4.6f\n", getDGain(&rollRatePidSettings));

		//Rate Pitch P gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_P]);
		setPGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Pitch P Gain=%4.6f\n", getPGain(&pitchRatePidSettings));
		//Rate Pitch I gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I]);
		setIGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Pitch I Gain=%4.6f\n", getIGain(&pitchRatePidSettings));
		//Rate Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I_LIMIT]);
		setILimit(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchRatePidSettings));
		//Rate Pitch D gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_D]);
		setDGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Pitch D Gain=%4.6f\n", getDGain(&pitchRatePidSettings));

		//Rate Yaw P gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_P]);
		setPGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Yaw P Gain=%4.6f\n", getPGain(&yawRatePidSettings));
		//Rate Yaw I gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I]);
		setIGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Yaw I Gain=%4.6f\n", getIGain(&yawRatePidSettings));
		//Rate Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I_LIMIT]);
		setILimit(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Yaw I Output Limit=%4.6f\n",
				getILimit(&yawRatePidSettings));
		//Rate Yaw D gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_D]);
		setDGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL,"Rate Yaw D Gain=%4.6f\n", getDGain(&yawRatePidSettings));

		break;

		default:
			
		_DEBUG(DEBUG_NORMAL,"unknow packet\n");

	}

	return true;

}



