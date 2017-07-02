/******************************************************************************
The radioControl.c in RaspberryPilot project is placed under the MIT license

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
#include "altHold.h"
#include "securityMechanism.h"

static int serialFd;
static pthread_t radioThreadId;
static pthread_t transmitThreadId;
static bool logIsEnable;
static unsigned long rev_success=0;
static unsigned long rev_drop=0;

void *radioReceiveThread(void *arg);
void *radioTransmitThread(void *arg);
short processRadioMessages(int fd, char *buf, short lenth);
bool extractPacketInfo(char *buf, int lenth,
		char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]);
bool checkLogIsEnable();
void setLogIsEnable(bool v);
bool checkPacketFieldIsValid(char *buf, short lenth);
unsigned int hexStringToInt(char * hexString, unsigned int len);
unsigned short getChecksum(char *buf, unsigned int len);

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
bool radioControlInit() {

	serialFd = 0;

	if ((serialFd = serialOpen("/dev/ttyAMA0", SERIAL_BAUDRATE)) < 0) {
		_DEBUG(DEBUG_NORMAL, "Unable to open serial device: %s\n",
				strerror(errno));
		return false;
	}

	serialFlush(serialFd);

	if (pthread_create(&transmitThreadId, NULL, radioTransmitThread,
			&serialFd)) {
		_DEBUG(DEBUG_NORMAL, "transmitThreadId create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL, "start transmiting...\n");
	}

	//start radio receiver thread
	if (pthread_create(&radioThreadId, NULL, radioReceiveThread, &serialFd)) {
		_DEBUG(DEBUG_NORMAL, "radioThreadId create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL, "start reveiveing...\n");
	}
	return true;
}

/**
 * check whether we should send more log to remote controler or not
 *
 * @param 
 * 		void
 *
 * @return
 *		bool
 *
 */
bool checkLogIsEnable(){
	return logIsEnable;
}

/**
 * whether we should send more log to remote controler or not
 *
 * @param 
 * 		v
 *
 * @return
 *		void
 *
 */

void setLogIsEnable(bool v){
	logIsEnable=v;
}

void printPayload(unsigned char *payload,unsigned int len){

	int j=0;
	for(j=0;j<len;j++){
		_DEBUG(DEBUG_NORMAL,"%x ", *(payload+j));
	}
	_DEBUG(DEBUG_NORMAL,"\n");
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
		 roll attitude: 0
		 pitch attitude: 1
		 yaw attitude: 2
		 height: 3
		 rollAttitudeSp: 4
		 pitchAttitudeSp: 5
		 yawAttitudeSp: 6
		 heightSp: 7
		 roll gyro: 8
		 pitch gyro: 9
		 yaw gyro: 10
		 center throttle: 11
		 ccw1 throttle: 12
		 cw1 throttle : 13
		 ccw2 throttle: 14
		 cw2 throttle: 15
		 #
		 */
		if(checkLogIsEnable()){
			snprintf(message, sizeof(message),
				"@%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d#",
				(int)getRoll(), (int)getPitch(), (int)getYaw(), (int)getCurrentAltHoldAltitude(),
				(int) getPidSp(&rollAttitudePidSettings),
				(int) getPidSp(&pitchAttitudePidSettings),
				(int) (getYawCenterPoint() + getPidSp(&yawAttitudePidSettings)),
				(int) getPidSp(&altHoldAltSettings), (int) getRollGyro(),
				(int) getPitchGyro(), (int) getYawGyro(),
				getThrottlePowerLevel(), getMotorPowerLevelCCW1(),
				getMotorPowerLevelCW1(), getMotorPowerLevelCCW2(),
				getMotorPowerLevelCW2());
		}else{
			snprintf(message, sizeof(message),
				"@%d:%d:%d:%d#",
				(int)getRoll(), (int)getPitch(), (int)getYaw(), (int)getCurrentAltHoldAltitude());
		}
		if ('#' != message[strlen(message) - 1]) {
			_DEBUG(DEBUG_NORMAL, "invilid package\n");
		} else {
			serialPuts(fd, message);
			increasePacketCounter();
			memset(message, '\0', sizeof(message));
		}

		usleep(TRANSMIT_TIMER);
	}

	pthread_exit((void *) 1234);
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

	while (!getLeaveFlyControlerFlag()) {

		if (serialDataAvail(fd)) {

			resetPacketCounter();

			getChar = serialGetchar(fd);

			if (getChar == '@') {
				if(count!=0){
					_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid: '#' is lost, buf=%s \n",buf);
					rev_drop++;
				}
				memset(buf, '\0', sizeof(buf));
				buf[0] = getChar;
				count = 1;
			} else if ((getChar == '#') && (buf[0] == '@')
					&& (count < sizeof(buf))) {
				buf[count] = getChar;
				processRadioMessages(fd, buf, count+1);
				memset(buf, '\0', sizeof(buf));
				count = 0;
			} else {
				if (buf[0] == '@' && (count < sizeof(buf))) {
					buf[count] = getChar;
					count++;
				} else {
					if(count!=0){
						rev_drop++;
						_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid: bufer overflow buf=%s getChar=%c\n",buf,getChar);
					}
					memset(buf, '\0', sizeof(buf));
					count = 0;
				}

			}
		}else{
			usleep(RECEIVE_TIMER);
		}
	}

	pthread_exit((void *) 1234);
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
bool extractPacketInfo(char *buf, int lenth,
		char container[PACKET_FIELD_NUM][PACKET_FIELD_LENGTH]) {

	int i = 0;
	char *token = NULL;
	char cmd[lenth];

	for (i = 1; (buf[i] != '#') && (i < lenth); i++) {
		cmd[i - 1] = buf[i];
	}
	cmd[i - 1] = '\0';

	//_DEBUG(DEBUG_NORMAL,"cmd=%s buf=%s lenth=%d\n",cmd,buf,lenth);

	
	i = 1;
	token = strtok(cmd, ":");
	strcpy(container[0], token);

	if (token == NULL)
		return false;

	do {
		token = strtok(NULL, ":");
		if (token == NULL) {
			break;
		} else {
			strcpy(container[i], token);
			i++;
		}
	} while (true);

	return true;

}

/**
 * get packet dropped rate
 *
 * @param 
 * 		void
 *
 * @return
 *		float
 */
void getPacketDropRate(){
	_DEBUG(DEBUG_NORMAL, "rev_ok/drop = %ld/%ld\n",rev_drop,rev_success);
}

/**
 * check whether packet is valid or not
 *
 * @param buf
 * 		received packet
 *
 * @param lenth
 * 		packet size
 *
 * @return
 *		bool
 */
bool checkPacketFieldIsValid(char *buf, short lenth){

	int i,j=0;
	int count=0;
	int count2=0;

	//_DEBUG(DEBUG_NORMAL, "length=%d %s \n",lenth,buf);

	//check header
	i=atoi(buf+1);

	if(!(i>HEADER_BEGIN&&i<HEADER_END)){
			_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid header: length=%d %s\n",lenth,buf);
			return false;
	}
	
	//check packet field
	for(j=0;j<lenth;j++){
		if(*(buf+j)==':'){
			count++;
		}
	}
	
	switch (i){
		case HEADER_ENABLE_FLY_SYSTEM:
			count2=ENABLE_FLY_SYSTEM_FIWLD_END-1;
			break;
		case HEADER_CONTROL_MOTION:
			count2=CONTROL_MOTION_END-1;
			break;
		case HEADER_HALT_PI:
			count2=HALT_PI_END-1;
			break;
		case HEADER_SETUP_FACTOR:
			count2=SETUP_FACTOR_END-1;
			break;
		case HEADER_SETUP_PID:
			count2=SETUP_PID_END-1;
			break;
		default:
			count2=-1;
	}

	if(count2!=count){
		_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid field: length=%d %s count2=%d count=%d\n",lenth,buf,count2,count);
		return false;
	}
	
	return true;
}


/**
 * convert a string  to int
 *
 * @param hexString
 * 		String
 *
 * @param len
 * 		length of payload
 *
 * @return
 * 		a number
 */
unsigned int hexStringToInt(char * hexString, unsigned int len){

	int i=0;
	unsigned int result=0;
	char hexTable[] = "0123456789ABCDEF";
	char *charPointer;

	for(i=0;i<len;i++){
		charPointer=strchr(hexTable,hexString[i]);
		if(NULL==charPointer){
			result=0x0;
			break;
		}
		result=(result<<4)+(charPointer-hexTable);
	}

	//_DEBUG(DEBUG_NORMAL,"%s %x\n",__func__,result);

	return result;
}

/**
 * calculate checksum from a payload
 *
 * @param buf
 * 		payload
 *
 * @param len
 * 		length of payload
 *
 * @return
 * 		checksum
 */
unsigned short getChecksum(char *buf,unsigned int len){

	int i;
	unsigned int checksunm = 0;

	for (i = 0; i < len; i = i + 2) {

		checksunm += (((buf[i] & 0xFF) << 8) | ((i + 1 < len) ? (buf[i + 1] & 0xFF) : 0x00));

		checksunm = ((checksunm & 0xFFFF) + ((checksunm >> 16)>0?1:0));
	}

	//_DEBUG(DEBUG_NORMAL,"%s %x\n",__func__,checksunm);
	
	return (unsigned short) checksunm;
}

/**
 * get index of checksum
 *
 * @param header
 * 		packet header
 *
 * @return
 * 		index
 */
unsigned short getChecksumFieldIndex(unsigned int header){

	unsigned short ret=0;
		
	switch (header){
		case HEADER_ENABLE_FLY_SYSTEM:
			ret=ENABLE_FLY_SYSTEM_CHECKSUM;
			break;
		case HEADER_CONTROL_MOTION:
			ret=CONTROL_MOTION_CHECKSUM;
			break;
		case HEADER_HALT_PI:
			ret=HALT_PI_CHECKSUM;
			break;
		case HEADER_SETUP_FACTOR:
			ret=SETUP_FACTOR_CHECKSUM;
			break;
		case HEADER_SETUP_PID:
			ret=SETUP_PID_CHECKSUM;
			break;
		default:
			ret=-1;
	}
	return ret;
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
	float yawShiftValue = 0;
	float throttlePercentage = 0.f;

#if CHECK_RECEIVER_PERIOD	
	struct timeval tv;
	struct timeval tv_last;
#endif

	if (!(checkPacketFieldIsValid(buf, lenth) && extractPacketInfo(buf, lenth, packet))){
		_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid field: %s\n",buf);
		rev_drop++;
		return false;
	}else if(!(getChecksum(buf,lenth-5)==(unsigned short)hexStringToInt(packet[getChecksumFieldIndex(atoi(packet[0]))],4))){
		rev_drop++;
		_DEBUG(DEBUG_RADIO_RX_FAIL, "invilid checksum: %s  getChecksum=0x%x hexStringToInt=0x%x\n",buf,getChecksum(buf,lenth-5),(unsigned short)hexStringToInt(packet[CONTROL_MOTION_CHECKSUM],4));
		return false;
	}else{
		rev_success++;
	}

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
		getPacketDropRate();
		break;

	case HEADER_CONTROL_MOTION:
		//control packet

#if CHECK_RECEIVER_PERIOD
		gettimeofday(&tv,NULL);
		_DEBUG("duration=%ld us\n",(tv.tv_sec-tv_last.tv_sec)*1000000+(tv.tv_usec-tv_last.tv_usec));
		tv_last.tv_usec=tv.tv_usec;
		tv_last.tv_sec=tv.tv_sec;
#endif

		rollSpShift = atof(packet[CONTROL_MOTION_ROLL_SP_SHIFT]);
		pitchSpShift = atof(packet[CONTROL_MOTION_PITCH_SP_SHIFT]);
		yawShiftValue = atof(packet[CONTROL_MOTION_YAW_SHIFT_VALUE]);
		throttlePercentage = atof(packet[CONTROL_MOTION_THROTTLE]);

		if (getAltHoldIsReady() && getEnableAltHold()) {

			//ALTHOLD:
			//1. get target altitude and assign it to pid controler
			//2. convert throttlePercentage to power level by target altitude

			setPidSp(&altHoldAltSettings,
					convertTargetAltFromeRemoteControler(
							(unsigned short) throttlePercentage));
			parameter = getDefaultPowerLevelWithTargetAlt();

		} else {

			throttlePercentage = throttlePercentage * 0.01f;
			parameter = getMinPowerLevel()
					+ (int) (throttlePercentage
							* (float) (getMaxPowerLeve() - getMinPowerLevel()));
		}

		if (parameter > getMaxPowerLeve() || parameter < getMinPowerLevel()) {
			_DEBUG(DEBUG_NORMAL, "invilid throttle level\n");
			break;
		}

		if (true == flySystemIsEnable()) {

			pthread_mutex_lock(&controlMotorMutex);

			setThrottlePowerLevel(parameter);

			if (getMinPowerLevel() == parameter) {
				
				resetPidRecord(&rollAttitudePidSettings);
				resetPidRecord(&pitchAttitudePidSettings);
				resetPidRecord(&yawAttitudePidSettings);
				resetPidRecord(&rollRatePidSettings);
				resetPidRecord(&pitchRatePidSettings);
				resetPidRecord(&yawRatePidSettings);
				resetPidRecord(&altHoldAltSettings);
				resetPidRecord(&altHoldlSpeedSettings);
				setYawCenterPoint(0.f);
				setAltStartPoint(0.f);
				setPidSp(&yawAttitudePidSettings, 321.0);
				setFlippingFlag(FLIP_NONE);

			} else {

				if (getPidSp(&yawAttitudePidSettings) == 321.0) {
					_DEBUG(DEBUG_NORMAL, "START Flying\n");
					setYawCenterPoint(getYaw());
					setAltStartPoint(getCurrentAltHoldAltitude());
					setPidSp(&yawAttitudePidSettings, 0);
				}
				
				setPidSp(&rollAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(rollSpShift, -getAngularLimit(),
								getAngularLimit()));
				setPidSp(&pitchAttitudePidSettings,
						LIMIT_MIN_MAX_VALUE(pitchSpShift, -getAngularLimit(),
								getAngularLimit()));
				setYawCenterPoint(getYawCenterPoint() + (yawShiftValue * 4));

				if(getFlippingIsEnable() && (FLIP_NONE == getFlippingFlag()) ){

					if(rollSpShift <= -getFlipThreadHold()){
						setFlippingFlag(FLIP_LEFT);
					}else if(rollSpShift >= getFlipThreadHold()){
						setFlippingFlag(FLIP_RIGHT);
					}

					if(pitchSpShift <= -getFlipThreadHold()){
						setFlippingFlag(FLIP_BACK);
					}else if(pitchSpShift >= getFlipThreadHold()){
						setFlippingFlag(FLIP_FRONT);
					}

					if(FLIP_NONE!=getFlippingFlag()){
						setFlippingStep(1);
					}
					
				}

				//_DEBUG(DEBUG_NORMAL,"setYawCenterPoint=%f\n",getYawCenterPoint());
			}
			pthread_mutex_unlock(&controlMotorMutex);

		}
		break;

	case HEADER_HALT_PI:
		//halt raspberry pi

		pthread_mutex_lock(&controlMotorMutex);
		setThrottlePowerLevel(0);
		setupAllMotorPoewrLevel(0, 0, 0, 0);
		disenableFlySystem();
		setLeaveFlyControlerFlag(true);
		pthread_mutex_unlock(&controlMotorMutex);
		system("sudo halt");
		break;

	case HEADER_SETUP_FACTOR:
		//setup factor

		//_DEBUG(DEBUG_NORMAL, "%s %d: %s\n", __func__, __LINE__, buf);
		/***/
		parameter = atoi(packet[SETUP_FACTOR_PERIOD]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPeriod(parameter);
		_DEBUG(DEBUG_NORMAL, "Adjustment Period: %d\n", getAdjustPeriod());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LEVEL_RANGE]);
		if (parameter == 0) {
			parameter = 1;
		}
		setAdjustPowerLeveRange(parameter);
		_DEBUG(DEBUG_NORMAL, "Adjustment Range: %d\n",
				getAdjustPowerLeveRange());
		/***/
		parameter = atoi(packet[SETUP_FACTOR_POWER_LIMIT]);
		if (parameter == 0) {
			parameter = 1;
		}
		setPidOutputLimitation(parameter);
		_DEBUG(DEBUG_NORMAL, "PID Output Limitation: %d\n",
				getPidOutputLimitation());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_GYRO_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setGyroLimit(parameterF);
		_DEBUG(DEBUG_NORMAL, "Angular Velocity Limit: %5.3f\n", getGyroLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ANGULAR_LIMIT]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setAngularLimit(parameterF);
		_DEBUG(DEBUG_NORMAL, "Roll/Pitch Angular Limit: %5.3f\n",
				getAngularLimit());
		/***/
		parameterF = atof(packet[SETUP_FACTOR_ROLL_CAL]);
		setPidSpShift(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Roll Angular Calibration: %5.3f\n",
				getPidSpShift(&rollAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_PITCH_CAL]);
		setPidSpShift(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Pitch Angular Calibration: %5.3f\n",
				getPidSpShift(&pitchAttitudePidSettings));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_0]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW1, parameterF);
		_DEBUG(DEBUG_NORMAL, "Motor 0 Gain: %5.3f\n",
				getMotorGain(SOFT_PWM_CCW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_1]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW1, parameterF);
		_DEBUG(DEBUG_NORMAL, "Motor 1 Gain: %5.3f\n",
				getMotorGain(SOFT_PWM_CW1));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_2]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CCW2, parameterF);
		_DEBUG(DEBUG_NORMAL, "Motor 2 Gain: %5.3f\n",
				getMotorGain(SOFT_PWM_CCW2));
		/***/
		parameterF = atof(packet[SETUP_FACTOR_MOTOR_GAIN_3]);
		if (parameterF == 0.) {
			parameterF = 1.;
		}
		setMotorGain(SOFT_PWM_CW2, parameterF);
		_DEBUG(DEBUG_NORMAL, "Motor 3 Gain: %5.3f\n",
				getMotorGain(SOFT_PWM_CW2));
		/***/
		parameter = atoi(packet[SETUP_FACTOR_VERTICAL_HOLD_ENABLE]);
		setEnableAltHold((bool) parameter);
		_DEBUG(DEBUG_NORMAL, "Enable aAltHold: %s\n",
				getEnableAltHold()==true?"true":"false");
		/***/
		parameterF = atof(
				packet[SETUP_FACTOR_ALTHOLD_ALT_PID_OUTPUT_LIMITATION]);
		setAltitudePidOutputLimitation(parameterF);
		_DEBUG(DEBUG_NORMAL, "getAltitudePidOutputLimitation: %5.3f\n",
				getAltitudePidOutputLimitation());
		/***/
		parameter = atoi(
				packet[SETUP_FACTOR_FLIP_ENABLED]);
		setFlippingIsEnable(parameter);
		_DEBUG(DEBUG_NORMAL, "getFlippingIsEnable: %d\n",
				getFlippingIsEnable());
		/***/
		parameter = atoi(
						packet[SETUP_FACTOR_LOG_ENABLED]);
				setLogIsEnable(parameter);
				_DEBUG(DEBUG_NORMAL, "checkLogIsEnable: %d\n",
						checkLogIsEnable());
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
		_DEBUG(DEBUG_NORMAL, "Attitude Roll P Gain=%4.6f\n",
				getPGain(&rollAttitudePidSettings));
		//attitude Roll I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I]);
		setIGain(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Roll I Gain=%4.6f\n",
				getIGain(&rollAttitudePidSettings));
		//attitude Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_I_LIMIT]);
		setILimit(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Roll I Output Limit=%4.6f\n",
				getILimit(&rollAttitudePidSettings));
		//attitude Roll D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_D]);
		setDGain(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Roll D Gain=%4.6f\n",
				getDGain(&rollAttitudePidSettings));
		//attitude Roll DB
		parameterF = atof(packet[SETUP_PID_ATTITUDE_ROLL_DB]);
		setPidDeadBand(&rollAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Roll DB=%4.6f\n",
				getPidDeadBand(&rollAttitudePidSettings));

		//attitude Pitch P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_P]);
		setPGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Pitch P Gain=%4.6f\n",
				getPGain(&pitchAttitudePidSettings));
		//attitude Pitch I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I]);
		setIGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Pitch I Gain=%4.6f\n",
				getIGain(&pitchAttitudePidSettings));
		//attitude Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_I_LIMIT]);
		setILimit(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchAttitudePidSettings));
		//attitude Pitch D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_D]);
		setDGain(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Pitch D Gain=%4.6f\n",
				getDGain(&pitchAttitudePidSettings));
		//attitude Pitch DB
		parameterF = atof(packet[SETUP_PID_ATTITUDE_PITCH_DB]);
		setPidDeadBand(&pitchAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Pitch DB=%4.6f\n",
				getPidDeadBand(&pitchAttitudePidSettings));

		//attitude Yaw P gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_P]);
		setPGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Yaw P Gain=%4.6f\n",
				getPGain(&yawAttitudePidSettings));
		//attitude Yaw I gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I]);
		setIGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Yaw I Gain=%4.6f\n",
				getIGain(&yawAttitudePidSettings));
		//attitude Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_I_LIMIT]);
		setILimit(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Yaw I Output Limit=%4.6f\n",
				getILimit(&yawAttitudePidSettings));
		//attitude Yaw D gain
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_D]);
		setDGain(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Yaw D Gain=%4.6f\n",
				getDGain(&yawAttitudePidSettings));
		//attitude Yaw DB
		parameterF = atof(packet[SETUP_PID_ATTITUDE_YAW_DB]);
		setPidDeadBand(&yawAttitudePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Attitude Yaw DB=%4.6f\n",
				getPidDeadBand(&yawAttitudePidSettings));

		//Rate Roll P gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_P]);
		setPGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Roll P Gain=%4.6f\n",
				getPGain(&rollRatePidSettings));
		//Rate Roll I gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I]);
		setIGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Roll I Gain=%4.6f\n",
				getIGain(&rollRatePidSettings));
		//Rate Roll I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_I_LIMIT]);
		setILimit(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Roll I Output Limit=%4.6f\n",
				getILimit(&rollRatePidSettings));
		//Rate Roll D gain
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_D]);
		setDGain(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Roll D Gain=%4.6f\n",
				getDGain(&rollRatePidSettings));
		//Rate Roll DB
		parameterF = atof(packet[SETUP_PID_RATE_ROLL_DB]);
		setPidDeadBand(&rollRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Roll DB=%4.6f\n",
				getPidDeadBand(&rollRatePidSettings));

		//Rate Pitch P gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_P]);
		setPGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Pitch P Gain=%4.6f\n",
				getPGain(&pitchRatePidSettings));
		//Rate Pitch I gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I]);
		setIGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Pitch I Gain=%4.6f\n",
				getIGain(&pitchRatePidSettings));
		//Rate Pitch I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_I_LIMIT]);
		setILimit(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Pitch I Output Limit=%4.6f\n",
				getILimit(&pitchRatePidSettings));
		//Rate Pitch D gain
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_D]);
		setDGain(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Pitch D Gain=%4.6f\n",
				getDGain(&pitchRatePidSettings));
		//Rate Pitch DB
		parameterF = atof(packet[SETUP_PID_RATE_PITCH_DB]);
		setPidDeadBand(&pitchRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Pitch DB=%4.6f\n",
				getPidDeadBand(&pitchRatePidSettings));

		//Rate Yaw P gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_P]);
		setPGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Yaw P Gain=%4.6f\n",
				getPGain(&yawRatePidSettings));
		//Rate Yaw I gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I]);
		setIGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Yaw I Gain=%4.6f\n",
				getIGain(&yawRatePidSettings));
		//Rate Yaw I outputLimit
		parameterF = atof(packet[SETUP_PID_RATE_YAW_I_LIMIT]);
		setILimit(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Yaw I Output Limit=%4.6f\n",
				getILimit(&yawRatePidSettings));
		//Rate Yaw D gain
		parameterF = atof(packet[SETUP_PID_RATE_YAW_D]);
		setDGain(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Yaw D Gain=%4.6f\n",
				getDGain(&yawRatePidSettings));
		//Rate Yaw DB
		parameterF = atof(packet[SETUP_PID_RATE_YAW_DB]);
		setPidDeadBand(&yawRatePidSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Rate Yaw DB=%4.6f\n",
				getPidDeadBand(&yawRatePidSettings));

		//Vertical Height P gain	
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_P]);
		setPGain(&altHoldAltSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Height P Gain=%4.6f\n",
				getPGain(&altHoldAltSettings));
		//Vertical Height I gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_I]);
		setIGain(&altHoldAltSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Height I Gain=%4.6f\n",
				getIGain(&altHoldAltSettings));
		//Vertical Height I outputLimit
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_I_LIMIT]);
		setILimit(&altHoldAltSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Height I Output Limit=%4.6f\n",
				getILimit(&altHoldAltSettings));
		//Vertical Height D gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_D]);
		setDGain(&altHoldAltSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Height D Gain=%4.6f\n",
				getDGain(&altHoldAltSettings));
		//Vertical Height DB
		parameterF = atof(packet[SETUP_PID_VERTICAL_HEIGHT_DB]);
		setPidDeadBand(&altHoldAltSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Height DB=%4.6f\n",
				getPidDeadBand(&altHoldAltSettings));

		//Vertical Speed P gain	
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_P]);
		setPGain(&altHoldlSpeedSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Speed P Gain=%4.6f\n",
				getPGain(&altHoldlSpeedSettings));
		//Vertical Speed I gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_I]);
		setIGain(&altHoldlSpeedSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Speed I Gain=%4.6f\n",
				getIGain(&altHoldlSpeedSettings));
		//Vertical Speed I outputLimit
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_I_LIMIT]);
		setILimit(&altHoldlSpeedSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Speed I Output Limit=%4.6f\n",
				getILimit(&altHoldlSpeedSettings));
		//Vertical Height D gain
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_D]);
		setDGain(&altHoldlSpeedSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Speed D Gain=%4.6f\n",
				getDGain(&altHoldlSpeedSettings));
		//Vertical Height DB
		parameterF = atof(packet[SETUP_PID_VERTICAL_SPEED_DB]);
		setPidDeadBand(&altHoldlSpeedSettings, parameterF);
		_DEBUG(DEBUG_NORMAL, "Vertical Speed DB=%4.6f\n",
				getPidDeadBand(&altHoldlSpeedSettings));

		break;

	default:

		_DEBUG(DEBUG_NORMAL, "unknow packet\n");

	}

	return true;

}

