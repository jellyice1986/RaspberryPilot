
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "commonLib.h"
#include "flyControler.h"
#include "pid.h"
#include "motorControl.h"
#include "systemControl.h"
#include "mpu6050.h"
#include "verticalHeightHold.h"
#include "radioControl.h"
#include "securityMechanism.h"

static int serialFd;
static pthread_t radioThreadId;
static pthread_t transmitThreadId;;


void *radioReceiveThread(void *arg);
void *radioTransmitThread(void *arg);
short processRadioMessages(int fd, char *buf, short lenth);
bool extractPacketInfo(char *buf,int lenth, char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]);


/**
* Init paramtes and states of radioControler 
*/
bool radioControlInit(){

	serialFd=0;

	if ((serialFd = serialOpen("/dev/ttyAMA0", SERIAL_BAUDRATE)) < 0) {
			printf("Unable to open serial device: %s\n", strerror(errno));
			return false;
		}
		serialFlush (serialFd);

		if (pthread_create(&transmitThreadId, NULL, radioTransmitThread, &serialFd)) {
			printf("transmitThreadId create failed\n");
			return false;
		} else {
			printf("start transmiting...\n");
		}

		//start radio receiver thread
		if (pthread_create(&radioThreadId, NULL, radioReceiveThread, &serialFd)) {
			printf("radioThreadId create failed\n");
			return false;
		} else {
			printf("start reveiveing...\n");
		}
		return true;
}


/**
 *  report pilot's information to remote side
 */
void *radioTransmitThread(void *arg) {

	char message[150];
	int fd = *(int *) arg;

#if 1
	while (!(true==getLeaveFlyControlerFlag())) {

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
		sprintf(message,
				"@%.1f:%.1f:%.1f:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
				getRoll(), getPitch(), getYaw(), (int)getPidSp(&rollAttitudePidSettings),
				(int)getPidSp(&pitchAttitudePidSettings),  (int)(getYawCenterPoint()+getPidSp(&yawAttitudePidSettings)),
				(int)getRollGyro(), (int)getPitchGyro(), (int)getYawGyro(),
				getThrottlePowerLevel(), getMotorPowerLevelCCW1(),
				getMotorPowerLevelCW1(), getMotorPowerLevelCCW2(),
				getMotorPowerLevelCW2());
		//printf("%s\n",message);
		if (strlen(message) > sizeof(message))
			printf("buffer overflow\n");
		
		serialPuts(fd, message);
		increasePacketCounter();
		memset(message, '\0', sizeof(message));
		usleep(TRANSMIT_TIMER);
	}
#endif
}

/**
 * This thread is used to receive packets from remote side
 */
void *radioReceiveThread(void *arg) {

	int fd = *(int *) arg;
	char buf[300];
	char getChar;
	char count = 0;

	memset(buf, '\0', sizeof(buf));

	while (!(true==getLeaveFlyControlerFlag())) {
		if (serialDataAvail(fd)) {
			getChar = serialGetchar(fd);
//			printf("get %c\n",getChar);
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
}


bool extractPacketInfo(char *buf,int lenth, char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]){

	int i=0;
	char *token=NULL;
	char cmd[lenth];

	//printf("%s\n",buf);
	
	for (i = 1; (buf[i] != '#')&&(i<lenth); i++) {
		cmd[i - 1] = buf[i];
	}
	cmd[i-1]='\0';
	i=1;
	token = strtok(cmd, ":");
	strncpy(container[0],token,sizeof(token));

	if(token==NULL) return false;
		
	do{
		token = strtok(NULL, ":");
		if(token==NULL){
			break; 
		}else{
			strncpy(container[i],token,sizeof(token));
			i++;
		}
	}while(true);
	
	return true;
		
}

/**
 * Decode a received packet
 *
 * @param buf
 * 				received packet
 *
 * @param lenth
 * 				packet size
 */
short processRadioMessages(int fd, char *buf, short lenth) {

	char packet[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH];
	short parameter = 0;
	float parameterF = 0.0;
	float rollSpShift = 0;
	float pitchSpShift = 0;
	float yawShiftValue=0;
	float throttlePercentage=0;
	struct timeval tv;
	struct timeval tv_last;

	resetPacketCounter();

	if(!extractPacketInfo(buf,lenth,packet)) return;


	switch (atoi(packet[0])) {
		
		case HEADER_ENABLE_FLY_SYSTEM:
			// enable fly system (init motor)
			// @1#

		if (1 == atoi(packet[ENABLE_FLY_SYSTEM_FIWLD_ISENABLE])) {
			// printf("1\n");
			if (!flySystemIsEnable()) {
				enableFlySystem();
				motorInit();
			}
		} else {
			//printf("0\n");
			disenableFlySystem();
		}

		break;
		
		case HEADER_CONTROL_MOTION:
		
#if 0 /*check cycle time of while*/
				gettimeofday(&tv,NULL);
				printf("duration=%d us\n",(tv.tv_sec-tv_last.tv_sec)*1000000+(tv.tv_usec-tv_last.tv_usec));
				tv_last.tv_usec=tv.tv_usec;
				tv_last.tv_sec=tv.tv_sec;
#endif
		// report throttle and attitude
		// @2 : throttle : roll sp shift : pitch sp shift : yaw shift value#

		//printf("%s\n",buf);

		rollSpShift = atof(packet[CONTROL_MOTION_ROLL_SP_SHIFT]);
		pitchSpShift = atof(packet[CONTROL_MOTION_PITCH_SP_SHIFT]);
		yawShiftValue = atof(packet[CONTROL_MOTION_YAW_SHIFT_VALUE]);
		throttlePercentage=atof(packet[CONTROL_MOTION_THROTTLE]) / 100.f;
		parameter = getMinPowerLevel()
				+ (int) (throttlePercentage
						* (float) (getMaxPowerLeve()
								- getMinPowerLevel())); 
		
		//printf("getMaxPowerLeveRange()- getMinPowerLeveRange()=%f\n",getMaxPowerLeve()- getMinPowerLevel());

		if (parameter > getMaxPowerLeve()
				|| parameter < getMinPowerLevel()) {
			printf("break\n");
			break;
		}
		
		if (true==flySystemIsEnable()) {
			
			pthread_mutex_lock(&controlMotorMutex);

#ifdef FEATURE_VH
			if(false==getVerticalHeightHoldEnable())
#endif
			setThrottlePowerLevel(parameter);
			
			if (getMinPowerLevel() == parameter) {
				//printf("STOP\n");
				resetPidRecord(&rollAttitudePidSettings);
				resetPidRecord(&pitchAttitudePidSettings);
				resetPidRecord(&yawAttitudePidSettings);
				resetPidRecord(&rollRatePidSettings);
				resetPidRecord(&pitchRatePidSettings);
				resetPidRecord(&yawRatePidSettings);
				resetPidRecord(&verticalHeightSettings);
				resetPidRecord(&verticalSpeedSettings);
				setYawCenterPoint(0);
				setPidSp(&yawAttitudePidSettings, 321.0);

			} else {
				if (getPidSp(&yawAttitudePidSettings) == 321.0) {
					printf("START Flying\n");
					setYawCenterPoint(getYaw());
					setPidSp(&yawAttitudePidSettings, 0);
#ifdef FEATURE_VH				
					setStartRisenVerticalHeight(getVerticalHeight());					
					setInitVHH(true);
#endif
				}
				setPidSp(&rollAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(rollSpShift, -getAngularLimit(),
								getAngularLimit()));
				setPidSp(&pitchAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(pitchSpShift, -getAngularLimit(),
								getAngularLimit()));
				setYawCenterPoint(getYawCenterPoint()+(yawShiftValue*1.0));
#ifdef FEATURE_VH				
				setPidSp(&verticalHeightSettings,(float)getStartRisenVerticalHeight()+(throttlePercentage*(float)getMaxRisenVerticalHeight()));
#endif
			}
			pthread_mutex_unlock(&controlMotorMutex);

		}

			//printf("throttle=%d roll=%f pitch=%f\n",parameter,rollSpShift,pitchSpShift);
		break;

	case HEADER_HALT_PI:
		//halt raspberry pi
		//@3#
		pthread_mutex_lock(&controlMotorMutex);
		setupAllMotorPoewrLevel(getMinPowerLevel(), getMinPowerLevel(),
				getMinPowerLevel(), getMinPowerLevel());
		pthread_mutex_unlock(&controlMotorMutex);
		system("sudo halt");
		setLeaveFlyControlerFlag(true);
		break;

	case HEADER_SETUP_FACTOR:

		printf("%s %d: %s\n", __func__, __LINE__, buf);
		/***/
		parameter = atoi(packet[SETUP_FACTOR_PERIOD]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPeriod(parameter);
		printf("Adjustment Period: %d\n", getAdjustPeriod());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LEVEL_RANGE]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPowerLeveRange(parameter);
		printf("Adjustment Range: %d\n", getAdjustPowerLeveRange());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LIMIT]);
		if (parameter == 0) {
			parameter = 1;
		}
		setPidOutputLimitation(parameter);
		printf("PID Output Limitation: %d\n", getPidOutputLimitation());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_GYRO_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setGyroLimit(parameterF);
		printf("Angular Velocity Limit: %5.3f\n", getGyroLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ANGULAR_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setAngularLimit(parameterF);
		printf("Roll/Pitch Angular Limit: %5.3f\n", getAngularLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ROLL_CAL]);
		setPidSpShift(&rollAttitudePidSettings, parameterF);
		printf("Roll Angular Calibration: %5.3f\n",
				getPidSpShift(&rollAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_PITCH_CAL]);
		setPidSpShift(&pitchAttitudePidSettings, parameterF);
		printf("Pitch Angular Calibration: %5.3f\n",
				getPidSpShift(&pitchAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_0]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW1, parameterF);
		printf("Motor 0 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_1]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW1, parameterF);
		printf("Motor 1 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_2]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW2, parameterF);
		printf("Motor 2 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW2));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_3]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW2, parameterF);
		printf("Motor 3 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW2));
		/***/
		parameter = atoi(packet[SETUP_FACTOR_VERTICAL_HOLD_ENABLE]);
#ifdef FEATURE_VH		
		setVerticalHeightHoldEnable((bool)parameter);
		printf("Vertical Height Hold Enable: %s\n", getVerticalHeightHoldEnable()==true?"true":"false");
#endif
		/***/
		break;

	case HEADER_OlED_DISPLAY: 
		// oled control, doesn't exist anymore
		break;

	case HEADER_SETUP_PID:
		printf("%s\n", buf);

		//attitude Roll P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_P]);
		setPGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll P Gain=%4.6f\n", getPGain(&rollAttitudePidSettings));

		//attitude Roll I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I]);
		setIGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll I Gain=%4.6f\n", getIGain(&rollAttitudePidSettings));

		//attitude Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I_LIMIT]);
		setILimit(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll I Output Limit=%4.6f\n",
				getILimit(&rollAttitudePidSettings));

		//attitude Roll D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_D]);
		setDGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll D Gain=%4.6f\n", getDGain(&rollAttitudePidSettings));

		//attitude Pitch P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_P]);
		setPGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch P Gain=%4.6f\n", getPGain(&pitchAttitudePidSettings));

		//attitude Pitch I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I]);
		setIGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch I Gain=%4.6f\n", getIGain(&pitchAttitudePidSettings));

		//attitude Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I_LIMIT]);
		setILimit(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchAttitudePidSettings));

		//attitude Pitch D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_D]);
		setDGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch D Gain=%4.6f\n", getDGain(&pitchAttitudePidSettings));

		//attitude Yaw P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_P]);
		setPGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw P Gain=%4.6f\n", getPGain(&yawAttitudePidSettings));

		//attitude Yaw I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I]);
		setIGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw I Gain=%4.6f\n", getIGain(&yawAttitudePidSettings));

		//attitude Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I_LIMIT]);
		setILimit(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw I Output Limit=%4.6f\n",
				getILimit(&yawAttitudePidSettings));

		//attitude Yaw D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_D]);
		setDGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw D Gain=%4.6f\n", getDGain(&yawAttitudePidSettings));

		//Rate Roll P gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_P]);
		setPGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll P Gain=%4.6f\n", getPGain(&rollRatePidSettings));
		//Rate Roll I gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I]);
		setIGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll I Gain=%4.6f\n", getIGain(&rollRatePidSettings));
		//Rate Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I_LIMIT]);
		setILimit(&rollRatePidSettings, parameterF);
		printf("Rate Roll I Output Limit=%4.6f\n",
				getILimit(&rollRatePidSettings));
		//Rate Roll D gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_D]);
		setDGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll D Gain=%4.6f\n", getDGain(&rollRatePidSettings));

		//Rate Pitch P gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_P]);
		setPGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch P Gain=%4.6f\n", getPGain(&pitchRatePidSettings));
		//Rate Pitch I gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I]);
		setIGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch I Gain=%4.6f\n", getIGain(&pitchRatePidSettings));
		//Rate Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I_LIMIT]);
		setILimit(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchRatePidSettings));
		//Rate Pitch D gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_D]);
		setDGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch D Gain=%4.6f\n", getDGain(&pitchRatePidSettings));

		//Rate Yaw P gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_P]);
		setPGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw P Gain=%4.6f\n", getPGain(&yawRatePidSettings));
		//Rate Yaw I gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I]);
		setIGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw I Gain=%4.6f\n", getIGain(&yawRatePidSettings));
		//Rate Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I_LIMIT]);
		setILimit(&yawRatePidSettings, parameterF);
		printf("Rate Yaw I Output Limit=%4.6f\n",
				getILimit(&yawRatePidSettings));
		//Rate Yaw D gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_D]);
		setDGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw D Gain=%4.6f\n", getDGain(&yawRatePidSettings));

		//Vertical Height P gain	
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_P]);
		setPGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height P Gain=%4.6f\n", getPGain(&verticalHeightSettings));		
		//Vertical Height I gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_I]);
		setIGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height I Gain=%4.6f\n", getIGain(&verticalHeightSettings));
		//Vertical Height I outputLimit
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_I_LIMIT]);
		setILimit(&verticalHeightSettings, parameterF);
		printf("Vertical Height I Output Limit=%4.6f\n",
		getILimit(&verticalHeightSettings));
		//Vertical Height D gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_D]);
		setDGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height D Gain=%4.6f\n", getDGain(&verticalHeightSettings));

		//Vertical Speed P gain	
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_P]);
		setPGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed P Gain=%4.6f\n", getPGain(&verticalSpeedSettings));
		//Vertical Speed I gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_I]);
		setIGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed I Gain=%4.6f\n", getIGain(&verticalSpeedSettings));
		//Vertical Speed I outputLimit
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_I_LIMIT]);
		setILimit(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed I Output Limit=%4.6f\n",
		getILimit(&verticalSpeedSettings));
		//Vertical Height D gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_D]);
		setDGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed D Gain=%4.6f\n", getDGain(&verticalSpeedSettings));
		
		break;
	}

}



