#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include "commonLib.h"
#include "flyControler.h"
#include "vl53l0x.h"
#include "motorControl.h"
#include "mpu6050.h"
#include "altHold.h"

#define ALTHOLD_CHECK_CYCLE_TIME 0
#define MODULE_TYPE ALTHOLD_MODULE_VL53L0X

static float aslRaw = 0.f;
static float asl = 0.f;
static float aslLong = 0.f;
static float aslAlpha = 0.3f;   	// Short term smoothing
static float aslLongAlpha = 0.6f;   	// Long term smoothing
static float altHoldAccSpeedAlpha = 0.5f;
static float altHoldSpeedAlpha = 0.6f;
static float altHoldSpeedDeadband = 0.f;
static float altHoldAccSpeedDeadband = 3.f;
static float altHoldSpeed = 0.f;
static float altHoldAccSpeed = 0.f;
static float altHoldAltSpeed = 0.f;
static float altHoldAccSpeedGain = 0.3f;
static float altHoldAltSpeedGain = 3.f;
static float factorForPowerLevelAndAlt = 15.f;
static bool altHoldIsReady = false;
static bool enableAltHold = false;
static bool altholdIsUpdate = false;
static unsigned short maxAlt = 50; 		//cm
static unsigned short startAlt = 0;  		//cm
static unsigned short targetAlt = 0;
static pthread_t altHoldThreadId;
static pthread_mutex_t altHoldIsUpdateMutex;

static void setAltHoldIsReady(bool v);
static void setMaxAlt(unsigned short v);
static unsigned short getMaxAlt();
static void setStartAlt(unsigned short v);
static unsigned short getStartAlt();
static void *altHoldThread(void *arg);
static float getAccWithoutGravity();
static void updateSpeedByAcceleration();
static void *altHoldUpdate(void *arg);

/**
 * init althold
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
bool initAltHold() {

	setAltHoldIsReady(false);

	switch (MODULE_TYPE) {

	case ALTHOLD_MODULE_VL53L0X:

		if (!vl53l0xInit()) {
			_DEBUG(DEBUG_NORMAL, "vl53l0x Init failed\n");
			return false;
		}

		setMaxAlt(150);  		//cm
		setStartAlt(0);  		//cm

		break;

	case ALTHOLD_MODULE_MS5611:

		//unavailable
		return false;

		break;

	default:

		return false;

		break;

	}

	if (pthread_mutex_init(&altHoldIsUpdateMutex, NULL) != 0) {
		_ERROR("(%s-%d) altHoldIsUpdateMutex init failed\n", __func__,
				__LINE__);
		return false;
	}

	if (pthread_create(&altHoldThreadId, NULL, altHoldUpdate, 0)) {
		_DEBUG(DEBUG_NORMAL, "altHold thread create failed\n");
		return false;
	} else {
		_DEBUG(DEBUG_NORMAL, "start altHold thread...\n");
	}

	setAltHoldIsReady(true);

	return true;
}

/**
 * get current altitude
 *
 * @param
 * 		void
 *
 * @return
 *		altHold
 *
 */
float getAsl(void) {
	return asl;
}

/**
 * get the flag to indicate whether enable althold or not
 *
 * @param
 * 		void
 *
 * @return
 *		altHold is ready or not
 *
 */
bool getEnableAltHold() {
	return enableAltHold;
}

/**
 * set the flag to indicate whether enable althold or not
 *
 * @param
 * 		altHold is ready or not
 *
 * @return
 *		void
 *
 */
void setEnableAltHold(bool v) {
	enableAltHold = v;
}

/**
 * get the flag to indicate whether althold is ready or not
 *
 * @param
 * 		void
 *
 * @return
 *		altHold is ready or not
 *
 */
bool getAltHoldIsReady() {
	return altHoldIsReady;
}

/**
 * set the flag to indicate whether althold is ready or not
 *
 * @param
 * 		altHold is ready or not
 *
 * @return
 *		void
 *
 */
void setAltHoldIsReady(bool v) {
	altHoldIsReady = v;
}

/**
 * set the maximum value of altitude
 *
 * @param
 * 		maximum value of altitude
 *
 * @return
 *		void
 *
 */
void setMaxAlt(unsigned short v) {
	maxAlt = v;
}

/**
 * get the maximum value of altitude
 *
 * @param
 * 		void
 *
 * @return
 *		maximum value of altitude
 *
 */
unsigned short getMaxAlt() {
	return maxAlt;
}

/**
 * set the start value of altitude
 *
 * @param
 * 		start value of altitude
 *
 * @return
 *		void
 *
 */
void setStartAlt(unsigned short v) {
	startAlt = v;
}

/**
 * get the start value of altitude
 *
 * @param
 * 		void
 *
 * @return
 *		start value of altitude
 *
 */
unsigned short getStartAlt() {
	return startAlt;
}

/**
 * calculates acceleration without gravity
 *
 * @param
 * 		void
 *
 * @return
 *		acceleration
 *
 */
float getAccWithoutGravity() {
	return (getXAcc() * getXGravity() + getYAcc() * getYGravity()
			+ getZAcc() * getZGravity() - 1.f);
}

/**
 * updates speed by acceleration
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void updateSpeedByAcceleration() {

	if (getAltHoldIsReady()) {

		altHoldAccSpeed = altHoldAccSpeed * altHoldAccSpeedAlpha
				+ (1.f - altHoldAccSpeedAlpha)
						* deadband(getAccWithoutGravity() * 100.f,
								altHoldAccSpeedDeadband) * altHoldAccSpeedGain;
		_DEBUG_HOVER(DEBUG_HOVER_ACC_SPEED, "(%s-%d) altHoldAccSpeed=%.3f\n",
				__func__, __LINE__, altHoldAccSpeed);
	}

}

/**
 * get current altitude
 *
 * @param
 * 		void
 *
 * @return
 *		altitude (cm)
 *
 */
float getCurrentAltHoldAltitude() {
	return asl;
}

/**
 * get current speed
 *
 * @param
 * 		void
 *
 * @return
 *		speed (cm/sec)
 *
 */
float getCurrentAltHoldSpeed() {
	return altHoldSpeed;
}

/**
 *  check whether update AltHold info or not
 *
 * @param
 * 		void
 *
 * @return
 *		true or false
 *
 */
bool updateAltHold() {

	bool ret = false;

	pthread_mutex_lock(&altHoldIsUpdateMutex);
	if (altholdIsUpdate) {
		altholdIsUpdate = false;
		ret = true;
	} else {
		ret = false;
	}
	pthread_mutex_unlock(&altHoldIsUpdateMutex);

	return ret;
}

/**
 *  AltHold thread, updates altitude and vertical speed
 *
 * @param
 * 		arg
 *
 * @return
 *		pointer
 *
 */
void *altHoldUpdate(void *arg) {

	unsigned short data = 0;
	bool result = false;

#if ALTHOLD_CHECK_CYCLE_TIME
	struct timeval tv;
	struct timeval tv2;
#endif

	while (!getLeaveFlyControlerFlag()) {

#if ALTHOLD_CHECK_CYCLE_TIME /*debug: check cycle time of this loop*/
		gettimeofday(&tv,NULL);
		_DEBUG(DEBUG_NORMAL,"duration=%ld us\n",(tv.tv_sec-tv2.tv_sec)*1000000+(tv.tv_usec-tv2.tv_usec));
		tv2.tv_usec=tv.tv_usec;
		tv2.tv_sec=tv.tv_sec;
#endif		

		if (getAltHoldIsReady()) {

			switch (MODULE_TYPE) {

			case ALTHOLD_MODULE_VL53L0X:

				result = vl53l0xGetMeasurementData(&data);
				aslRaw = (float) data * 0.1f;

				break;

			case ALTHOLD_MODULE_MS5611:
				//unavailable
				result = false;
				break;

			default:
				result = false;
				break;
			}

			if (result) {

				updateSpeedByAcceleration();

				asl = asl * aslAlpha + aslRaw * (1.f - aslAlpha);
				aslLong = aslLong * aslLongAlpha
						+ aslRaw * (1.f - aslLongAlpha);
				altHoldAltSpeed = deadband((asl - aslLong),
						altHoldSpeedDeadband) * altHoldAltSpeedGain;
				altHoldSpeed = altHoldAltSpeed * altHoldSpeedAlpha
						+ altHoldAccSpeed * (1.f - altHoldSpeedAlpha);

				pthread_mutex_lock(&altHoldIsUpdateMutex);
				altholdIsUpdate = true;
				pthread_mutex_unlock(&altHoldIsUpdateMutex);

				_DEBUG_HOVER(DEBUG_HOVER_ALT_SPEED,
						"(%s-%d) altHoldAltSpeed=%.3f\n", __func__, __LINE__,
						altHoldAltSpeed);
				_DEBUG_HOVER(DEBUG_HOVER_SPEED, "(%s-%d) altHoldSpeed=%.3f\n",
						__func__, __LINE__, altHoldSpeed);
				_DEBUG_HOVER(DEBUG_HOVER_RAW_ALTITUDE, "(%s-%d) aslRaw=%.3f\n",
						__func__, __LINE__, aslRaw);
				_DEBUG_HOVER(DEBUG_HOVER_FILTERED_ALTITUDE,
						"(%s-%d) asl=%.3f aslLong=%.3f\n", __func__, __LINE__,
						asl, aslLong);

			} else {
				usleep(500);
			}
		}
	}

	pthread_exit((void *) 1234);

}

/**
 *  convert the percentage of throttle  which is receive from
 *  remote controler into targer altitude
 *
 * @param
 * 		 percentage of throttle (1 to 100)
 *
 * @return
 *		target altitude
 *
 */
unsigned short convertTargetAltFromeRemoteControler(unsigned short v) {

	targetAlt = LIMIT_MIN_MAX_VALUE(LIMIT_MIN_MAX_VALUE(v,0,100)*5, 0,
			getMaxAlt());
	return targetAlt;
}

/**
 *  get a defeault power level with a altitude, you should make the factor and your Raspberry Pilot match well
 *
 * @param
 * 		 altitude
 *
 * @return
 *		power level
 *
 */
unsigned short getDefaultPowerLevelWithTargetAlt() {

	unsigned short ret = 0;

	ret =
			LIMIT_MIN_MAX_VALUE(
					MIN_POWER_LEVEL+(unsigned short)((float)targetAlt*factorForPowerLevelAndAlt),
					MIN_POWER_LEVEL, MAX_POWER_LEVEL);

	return ret;
}

