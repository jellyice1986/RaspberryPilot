
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "flyControler.h"
#include "pid.h"
#include "motorControl.h"
#include "systemControl.h"
#include "mpu6050.h"
#include "verticalHeightHold.h"
#include "radioControl.h"
#include "safeMachenism.h"




#define TRANSMIT_TIMER 50000

static int serialFd;
static pthread_t radioThreadId;
static pthread_t transmitThreadId;;


void *radioReceiveThread(void *arg);
void *radioTransmitThread(void *arg);
short processRadioMessages(int fd, char *buf, short lenth);


/**
* Init paramtes and states of radioControler 
*/
bool radioControlInit(){

	serialFd=0;

	if ((serialFd = serialOpen("/dev/ttyAMA0", 57600)) < 0) {
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
 * This thread is used to report runtime states of pilot to remote side
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
				"@%3.2f:%3.2f:%3.2f:%3.2f:%3.2f:%3.2f:%3.2f:%3.2f:%3.2f:%d:%d:%d:%d:%d#",
				getRoll(), getPitch(), getYaw(), getPidSp(&rollAttitudePidSettings),
				getPidSp(&pitchAttitudePidSettings),  getYawCenterPoint()+getPidSp(&yawAttitudePidSettings),
				getRollGyro(), getPitchGyro(), getYawGyro(),
				getThrottlePowerLevel(), getMotorPowerLevelCCW1(),
				getMotorPowerLevelCW1(), getMotorPowerLevelCCW2(),
				getMotorPowerLevelCW2());
		//printf("%s\n",message);
		if (strlen(message) > sizeof(message))
			printf("buffer overflow\n");
		serialPuts(fd, message);
		increasePacketAccCounter();
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
	short cmdIndex = -1;
	char cmd[lenth];
	char *token;
	short i = 0;
	short changeLeve = 0;
	short parameter = 0;
	float parameterF = 0.0;
	float rollSpShift = 0;
	float pitchSpShift = 0;
	float yawShiftValue=0;
	float throttlePercentage=0;
	struct timeval tv;
	struct timeval tv_last;


	resetPacketAccCounter();
	
	//printf("%s %d: %s\n",__func__,__LINE__,buf);

	memset(cmd, '\0', sizeof(cmd));

	for (i = 1; buf[i] != '#'; i++) {
		cmd[i - 1] = buf[i];
	}
//	printf("%s %d: %s\n",__func__,__LINE__,cmd);

	token = strtok(cmd, ":");
	cmdIndex = atoi(token);

	switch (cmdIndex) {
	case 1:
		// enable fly sysmon (init motor)
		// @1#
		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameter = atoi(token);
		if (1 == parameter) {
			// printf("1\n");
			if (false==flySystemIsEnable()) {
				enableFlySystem();
				motorInit();
			}
		} else {
			//printf("0\n");
			disenableFlySystem();
		}

		break;
	case 2:
		
#if 0 /*check cycle time of while*/
				gettimeofday(&tv,NULL);
				printf("duration=%d us\n",(tv.tv_sec-tv_last.tv_sec)*1000000+(tv.tv_usec-tv_last.tv_usec));
				tv_last.tv_usec=tv.tv_usec;
				tv_last.tv_sec=tv.tv_sec;
#endif
		// report throttle and attitude
		// @2 : throttle : roll sp shift : pitch sp shift : yaw shift value#

		//printf("%s\n",buf);

		//change motor power level
		token = strtok(NULL, ":");
//			printf("token=%s\n",token);
		if (token == 0) {
			break;
		}
		parameter = atoi(token);
		token = strtok(NULL, ":");
		rollSpShift = atof(token);

		token = strtok(NULL, ":");
		pitchSpShift = atof(token);

		token = strtok(NULL, ":");
		yawShiftValue = atof(token);
		//printf("factor=%d\n",(int)(((float)(parameter-100)/(float)100)*(float)(MAX_POWER_LEVEL-MIN_POWER_LEVEL)));
		//printf("parameter=%d\n",parameter);
		throttlePercentage=(float)parameter / 100.f;
		parameter = getMinPowerLevel()
				+ (int) (throttlePercentage
						* (float) (getMaxPowerLeve()
								- getMinPowerLevel())); /*(100~200)*10=1000~2000 us*/
		//printf("getMaxPowerLeveRange()- getMinPowerLeveRange()=%f\n",getMaxPowerLeve()- getMinPowerLevel());
		//printf("parameter=%d\n",parameter);

		if (parameter > getMaxPowerLeve()
				|| parameter < getMinPowerLevel()) {
			printf("break\n");
			break;
		}
		if (true==flySystemIsEnable()) {
			
			pthread_mutex_lock(&controlMotorMutex);

			//if(false==getVerticalHeightHoldEnable()){
			setThrottlePowerLevel(parameter);
			//}
			
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
					setStartRisenVerticalHeight(getVerticalHeight());
					setInitVHH(true);
				}
				setPidSp(&rollAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(rollSpShift, -getAngularLimit(),
								getAngularLimit()));
				setPidSp(&pitchAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(pitchSpShift, -getAngularLimit(),
								getAngularLimit()));
				setYawCenterPoint(getYawCenterPoint()+(yawShiftValue*4.0));
				setPidSp(&verticalHeightSettings,(float)getStartRisenVerticalHeight()+(throttlePercentage*(float)getMaxRisenVerticalHeight()));
				//printf("start asl: %d, target asl: %d Difference=%d\n",getStartRisenVerticalHeight(),(short)getPidSp(&verticalHeightSettings),(short)getPidSp(&verticalHeightSettings)-getStartRisenVerticalHeight());
				//yawShiftValue*4: 4 is a gain, yawShiftValue is 1 or -1
				//printf("getYawCenterPoint=%3.3f\n",getYawCenterPoint());
			}
			//printf("rollSpShift=%5.3f pitchSpShift=%5.3f\n",rollSpShift,pitchSpShift);
			//printf("ROLL SP: %5.3f SHIFT: %5.3f\n",getPidSp(&rollAttitudePidSettings),getPidSpShift(&rollAttitudePidSettings));
			//printf("PITCH SP: %5.3f SHIFT: %5.3f\n",getPidSp(&pitchPidSettings),getPidSpShift(&pitchPidSettings));
			pthread_mutex_unlock(&controlMotorMutex);

		}

			//printf("throttle=%d roll=%f pitch=%f\n",parameter,rollSpShift,pitchSpShift);

		break;

	case 3:
		//halt raspberry pi
		//@3#
		pthread_mutex_lock(&controlMotorMutex);
		setupAllMotorPoewrLevel(getMinPowerLevel(), getMinPowerLevel(),
				getMinPowerLevel(), getMinPowerLevel());
		pthread_mutex_unlock(&controlMotorMutex);
		system("sudo halt");
		setLeaveFlyControlerFlag(true);
		break;

	case 4:
		printf("%s %d: %s\n", __func__, __LINE__, buf);

		printf("%s\n", buf);
		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameter = atoi(token);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPeriod(parameter);
		printf("Adjustment Period: %d\n", getAdjustPeriod());
		/***/

		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameter = atoi(token);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPowerLeveRange(parameter);
		printf("Adjustment Range: %d\n", getAdjustPowerLeveRange());
		/***/

		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameter = atoi(token);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPowerLimit(parameter);
		printf("Adjustment Limit: %d\n", getAdjustPowerLimit());
		/***/

		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setGyroLimit(parameterF);
		printf("Angular Velocity Limit: %5.3f\n", getGyroLimit());
		/***/

		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setAngularLimit(parameterF);
		printf("Roll/Pitch Angular Limit: %5.3f\n", getAngularLimit());
		/***/

		token = strtok(NULL, ":");
		if (token == 0) {
			break;
		}
		parameterF = atof(token);
		setPidSpShift(&rollAttitudePidSettings, parameterF);
		printf("Roll Angular Calibration: %5.3f\n",
				getPidSpShift(&rollAttitudePidSettings));
		/***/

		token = strtok(NULL, ":");
                if (token == 0) {
                        break;
                }
		parameterF = atof(token);
		setPidSpShift(&pitchAttitudePidSettings, parameterF);
		printf("Pitch Angular Calibration: %5.3f\n",
				getPidSpShift(&pitchAttitudePidSettings));

		/***/

		token = strtok(NULL, ":");
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setMotorGain(SOFT_PWM_CCW1, parameterF);
		printf("Motor 0 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW1));
		/***/

		token = strtok(NULL, ":");
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setMotorGain(SOFT_PWM_CW1, parameterF);
		printf("Motor 1 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW1));

		/***/

		token = strtok(NULL, ":");
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setMotorGain(SOFT_PWM_CCW2, parameterF);
		printf("Motor 2 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CCW2));

		/***/

		token = strtok(NULL, ":");
		parameterF = atof(token);
		if (parameterF == 0) {
			parameterF = 1;
		}
		setMotorGain(SOFT_PWM_CW2, parameterF);
		printf("Motor 3 Gain: %5.3f\n", getMotorGain(SOFT_PWM_CW2));

		/***/
		
		token = strtok(NULL, ":");
		parameter = atoi(token);
		setVerticalHeightHoldEnable((bool)parameter);
		printf("Vertical Height Hold Enable: %s\n", getVerticalHeightHoldEnable()==true?"true":"false");

		break;

	case 5: 
		// oled control, doesn't exist anymore
		break;

	case 6:
		printf("%s\n", buf);

		//attitude Roll P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll P Gain=%4.6f\n", getPGain(&rollAttitudePidSettings));

		//attitude Roll I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll I Gain=%4.6f\n", getIGain(&rollAttitudePidSettings));

		//attitude Roll I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll I Output Limit=%4.6f\n",
				getILimit(&rollAttitudePidSettings));

		//attitude Roll D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&rollAttitudePidSettings, parameterF);
		printf("Attitude Roll D Gain=%4.6f\n", getDGain(&rollAttitudePidSettings));

		//attitude Pitch P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch P Gain=%4.6f\n", getPGain(&pitchAttitudePidSettings));

		//attitude Pitch I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch I Gain=%4.6f\n", getIGain(&pitchAttitudePidSettings));

		//attitude Pitch I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchAttitudePidSettings));

		//attitude Pitch D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&pitchAttitudePidSettings, parameterF);
		printf("Attitude Pitch D Gain=%4.6f\n", getDGain(&pitchAttitudePidSettings));

		//attitude Yaw P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw P Gain=%4.6f\n", getPGain(&yawAttitudePidSettings));

		//attitude Yaw I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw I Gain=%4.6f\n", getIGain(&yawAttitudePidSettings));

		//attitude Yaw I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw I Output Limit=%4.6f\n",
				getILimit(&yawAttitudePidSettings));

		//attitude Yaw D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&yawAttitudePidSettings, parameterF);
		printf("Attitude Yaw D Gain=%4.6f\n", getDGain(&yawAttitudePidSettings));

		//Rate Roll P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll P Gain=%4.6f\n", getPGain(&rollRatePidSettings));
		//Rate Roll I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll I Gain=%4.6f\n", getIGain(&rollRatePidSettings));
		//Rate Roll I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&rollRatePidSettings, parameterF);
		printf("Rate Roll I Output Limit=%4.6f\n",
				getILimit(&rollRatePidSettings));
		//Rate Roll D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&rollRatePidSettings, parameterF);
		printf("Rate Roll D Gain=%4.6f\n", getDGain(&rollRatePidSettings));

		//Rate Pitch P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch P Gain=%4.6f\n", getPGain(&pitchRatePidSettings));
		//Rate Pitch I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch I Gain=%4.6f\n", getIGain(&pitchRatePidSettings));
		//Rate Pitch I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchRatePidSettings));
		//Rate Pitch D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&pitchRatePidSettings, parameterF);
		printf("Rate Pitch D Gain=%4.6f\n", getDGain(&pitchRatePidSettings));

		//Rate Yaw P gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw P Gain=%4.6f\n", getPGain(&yawRatePidSettings));
		//Rate Yaw I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw I Gain=%4.6f\n", getIGain(&yawRatePidSettings));
		//Rate Yaw I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&yawRatePidSettings, parameterF);
		printf("Rate Yaw I Output Limit=%4.6f\n",
				getILimit(&yawRatePidSettings));
		//Rate Yaw D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&yawRatePidSettings, parameterF);
		printf("Rate Yaw D Gain=%4.6f\n", getDGain(&yawRatePidSettings));

		//Vertical Height P gain	
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height P Gain=%4.6f\n", getPGain(&verticalHeightSettings));		
		//Vertical Height I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height I Gain=%4.6f\n", getIGain(&verticalHeightSettings));
		//Vertical Height I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&verticalHeightSettings, parameterF);
		printf("Vertical Height I Output Limit=%4.6f\n",
		getILimit(&verticalHeightSettings));
		//Vertical Height D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&verticalHeightSettings, parameterF);
		printf("Vertical Height D Gain=%4.6f\n", getDGain(&verticalHeightSettings));

		//Vertical Speed P gain	
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setPGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed P Gain=%4.6f\n", getPGain(&verticalSpeedSettings));
		//Vertical Speed I gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setIGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed I Gain=%4.6f\n", getIGain(&verticalSpeedSettings));
		//Vertical Speed I outputLimit
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setILimit(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed I Output Limit=%4.6f\n",
		getILimit(&verticalSpeedSettings));
		//Vertical Height D gain
		token = strtok(NULL, ":");
		parameterF = atof(token);
		setDGain(&verticalSpeedSettings, parameterF);
		printf("Vertical Speed D Gain=%4.6f\n", getDGain(&verticalSpeedSettings));
		
		break;
	}

}


