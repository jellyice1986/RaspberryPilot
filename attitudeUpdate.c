/******************************************************************************
The attitudeUpdate.c in RaspberryPilot project is placed under the MIT license

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
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include "cJSON.h"
#include "commonLib.h"
#include "ahrs.h"
#include "smaFilter.h"
#include "flyControler.h"
#include "mpu6050.h"
#include "attitudeUpdate.h"

#define CHECK_ATTITUDE_UPDATE_LOOP_TIME 0

static bool attitudeIsInit;
static float verticalAcceleration;
static float xAcceleration;
static float yAcceleration;
static float yaw;
static float pitch;
static float roll;
static float yawGyro;
static float pitchGyro;
static float rollGyro;
static float xAcc;
static float yAcc;
static float zAcc;
static float xGravity;
static float yGravity;
static float zGravity;
static short imuRawData[9];
#ifdef MPU6050_9AXIS
// Hard iron calibration matrix
float mag_hard_iron_cal[3];
// Soft iron calibration matrix
float mag_soft_iron_cal[3][3];
static SMA_STRUCT x_magnetSmaFilterEntry;
static SMA_STRUCT y_magnetSmaFilterEntry;
static SMA_STRUCT z_magnetSmaFilterEntry;
#endif

void *attitudeUpdateThread();
static void GetXComponent(float *x, float *q);
static void GetYComponent(float *y, float *q);
static void GetZComponent(float *z, float *q);
static unsigned char GetYawPitchRoll(float *data, float *q, float *gravity);
static void getYawPitchRollInfo(float *yprAttitude, float *yprRate,
		float *xyzAcc, float *xComponent, float *yComponent, float *zComponent, float *xyzMagnet);

/**
 * init paramtes and states for attitudeUpdate
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
bool altitudeUpdateInit() {

	int calCount;
	
	attitudeIsInit=false;

	if(!parseMagnetCalibrationData(&calCount, mag_hard_iron_cal, mag_soft_iron_cal)){
		_DEBUG(DEBUG_NORMAL,"Read magnet calibration data fail\n");
		return false;
	}

	if(0 == calCount){
		_DEBUG(DEBUG_NORMAL,"Use default magnet calibration data\n");
	}
	
	_DEBUG(DEBUG_NORMAL,"Hard Iron: \n[%.3f %.3f %.3f]\n",mag_hard_iron_cal[0],mag_hard_iron_cal[1],mag_hard_iron_cal[2]);
	_DEBUG(DEBUG_NORMAL,"Soft Iron: \n[%.3f %.3f %.3f]\n[%.3f %.3f %.3f]\n[%.3f %.3f %.3f]\n",
		mag_soft_iron_cal[0][0],mag_soft_iron_cal[0][1],mag_soft_iron_cal[0][2],
		mag_soft_iron_cal[1][0],mag_soft_iron_cal[1][1],mag_soft_iron_cal[1][2],
		mag_soft_iron_cal[2][0],mag_soft_iron_cal[2][1],mag_soft_iron_cal[2][2]);

#ifdef MPU6050_9AXIS
	initSmaFilterEntity(&x_magnetSmaFilterEntry,"X_MAGNET",2);
	initSmaFilterEntity(&y_magnetSmaFilterEntry,"Y_MAGNET",2);
	initSmaFilterEntity(&z_magnetSmaFilterEntry,"Z_MAGNET",2);
#endif	

	attitudeIsInit=true;

	return true;
	
}

/**
 * update attitude
 *
 * @param 
 * 		void
 *
 * @return
 *		void
 *
 */
void attitudeUpdate(){

	float yrpAttitude[3];
	float pryRate[3];
	float xyzAcc[3];
	float xComponent[3];
	float yComponent[3];
	float zComponent[3];
	float xyzMagnet[3];
#if CHECK_ATTITUDE_UPDATE_LOOP_TIME
	struct timeval tv_c;
	static struct timeval tv_l;
	unsigned long timeDiff=0;
	
	gettimeofday(&tv_c,NULL);
	timeDiff=GET_USEC_TIMEDIFF(tv_c,tv_l);
	_DEBUG(DEBUG_NORMAL,"attitude update duration=%ld us\n",timeDiff);
	UPDATE_LAST_TIME(tv_c,tv_l);
#endif	
			
	getYawPitchRollInfo(yrpAttitude, pryRate, xyzAcc, xComponent, yComponent, zComponent,xyzMagnet); 

	setYaw(yrpAttitude[0]);
	setRoll(yrpAttitude[1]);
	setPitch(yrpAttitude[2]);
	setYawGyro(-pryRate[2]);
	setPitchGyro(pryRate[0]);
	setRollGyro(-pryRate[1]);
	setXAcc(xyzAcc[0]);
	setYAcc(xyzAcc[1]);
	setZAcc(xyzAcc[2]);
	setXGravity(zComponent[0]);
	setYGravity(zComponent[1]);
	setZGravity(zComponent[2]);
	setVerticalAcceleration(deadband((getXAcc() * zComponent[0] + getYAcc() * zComponent[1]
		+ getZAcc() * zComponent[2] - 1.f) * 100.f,3.f) );
	setXAcceleration(deadband((getXAcc() * xComponent[0] + getYAcc() * xComponent[1]
		+ getZAcc() * xComponent[2]) * 100.f,3.f) );
	setYAcceleration(deadband((getXAcc() * yComponent[0] + getYAcc() * yComponent[1]
		+ getZAcc() * yComponent[2]) * 100.f,3.f) );

	_DEBUG(DEBUG_ATTITUDE,
			"(%s-%d) ATT: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n", __func__,
			__LINE__, getRoll(), getPitch(), getYaw());
	_DEBUG(DEBUG_GYRO,
			"(%s-%d) GYRO: Roll=%3.3f Pitch=%3.3f Yaw=%3.3f\n",
			__func__, __LINE__, getRollGyro(), getPitchGyro(),
			getYawGyro());
	_DEBUG(DEBUG_ACC, "(%s-%d) ACC: x=%3.3f y=%3.3f z=%3.3f\n",
			__func__, __LINE__, getXAcc(), getYAcc(), getZAcc());
	
}

/**
 * set yaw
 *
 * @param t_yaw
 * 		yaw
 *
 * @return
 *		void
 *
 */
void setYaw(float t_yaw) {
	yaw = t_yaw;
}

/**
 * set pitch
 *
 * @param t_pitch
 * 		pitch
 *
 * @return
 *		void
 *
 */
void setPitch(float t_pitch) {
	pitch = t_pitch;
}

/**
 * set roll
 *
 * @param t_roll
 * 		roll
 *
 * @return
 *		void
 *
 */
void setRoll(float t_roll) {
	roll = t_roll;
}

/**
 * get yaw
 *
 * @param
 * 		void
 *
 * @return
 *		yaw
 *
 */
float getYaw() {
	return yaw;
}

/**
 * get Pitch
 *
 * @param
 * 		void
 *
 * @return
 *		pitch
 *
 */
float getPitch() {
	return pitch;
}

/**
 * get roll
 *
 * @param
 * 		void
 *
 * @return
 *		roll
 *
 */
float getRoll() {
	return roll;
}

float getVerticalAcceleration(){
	return verticalAcceleration;
}

void setVerticalAcceleration(float v){
	verticalAcceleration=v;
}

float getXAcceleration(){
	return xAcceleration;
}

void setXAcceleration(float v){
	xAcceleration=v;
}

float getYAcceleration(){
	return yAcceleration;
}

void setYAcceleration(float v){
	yAcceleration=v;
}


/**
 * set angular velocity of yaw
 *
 * @param t_yaw_gyro
 * 		angular velocity of yaw
 *
 * @return
 *		void
 *
 */
void setYawGyro(float t_yaw_gyro) {
	yawGyro = t_yaw_gyro;
}

/**
 * set angular velocity of pitch
 *
 * @param t_pitch_gyro
 * 		angular velocity of pitch
 *
 * @return
 *		void
 *
 */
void setPitchGyro(float t_pitch_gyro) {
	pitchGyro = t_pitch_gyro;
}

/**
 * set angular velocity of roll
 *
 * @param t_roll_gyro
 * 		angular velocity of roll
 *
 * @return
 *		void
 *
 */
void setRollGyro(float t_roll_gyro) {
	rollGyro = t_roll_gyro;
}

/**
 * get angular velocity of yaw
 *
 * @param
 * 		void
 *
 * @return
 *		angular velocity of yaw
 *
 */
float getYawGyro() {
	return yawGyro;
}

/**
 * get angular velocity of pitch
 *
 * @param
 * 		void
 *
 * @return
 *		angular velocity of pitch
 *
 */
float getPitchGyro() {
	return pitchGyro;
}

/**
 * get angular velocity of roll
 *
 * @param
 * 		void
 *
 * @return
 *		angular velocity of roll
 *
 */
float getRollGyro() {
	return rollGyro;
}

/**
 * set gravity of x axis
 *
 * @param x_gravity
 * 		gravity of x axis
 *
 * @return
 *		void
 *
 */
void setXGravity(float x_gravity) {
	xGravity = x_gravity;
}

/**
 * set gravity of y axis
 *
 * @param y_gravity
 * 		gravity of y axis
 *
 * @return
 *		void
 *
 */
void setYGravity(float y_gravity) {
	yGravity = y_gravity;
}

/**
 * set gravity of z axis
 *
 * @param z_gravity
 * 		gravity of z axis
 *
 * @return
 *		void
 *
 */
void setZGravity(float z_gravity) {
	zGravity = z_gravity;
}

/**
 * get gravity of x axis
 *
 * @param
 * 		void
 *
 * @return
 *		gravity of x axis
 *
 */
float getXGravity() {
	return xGravity;
}

/**
 * get gravity of y axis
 *
 * @param
 * 		void
 *
 * @return
 *		gravity of y axis
 *
 */
float getYGravity() {
	return yGravity;
}

/**
 * get gravity of z axis
 *
 * @param
 * 		void
 *
 * @return
 *		gravity of z axis
 *
 */
float getZGravity() {
	return zGravity;
}

/**
 * set accelerate of x axis
 *
 * @param x_acc
 * 		accelerate of x axis
 *
 * @return
 *		void
 *
 */
void setXAcc(float x_acc) {
	xAcc = x_acc;
}

/**
 * set accelerate of y axis
 *
 * @param y_acc
 * 		accelerate of y axis
 *
 * @return
 *		void
 *
 */
void setYAcc(float y_acc) {
	yAcc = y_acc;
}

/**
 * set accelerate of z axis
 *
 * @param z_acc
 * 		accelerate of z axis
 *
 * @return
 *		void
 *
 */
void setZAcc(float z_acc) {
	zAcc = z_acc;
}

/**
 * get accelerate of x axis
 *
 * @param
 * 		void
 *
 * @return
 *		accelerate of x axis
 *
 */
float getXAcc() {
	return xAcc;
}

/**
 * get accelerate of y axis
 *
 * @param
 * 		void
 *
 * @return
 *		accelerate of y axis
 *
 */
float getYAcc() {
	return yAcc;
}

/**
 * get accelerate of z axis
 *
 * @param
 * 		void
 *
 * @return
 *		accelerate of z axis
 *
 */
float getZAcc() {
	return zAcc;
}

/**
 * get yaw, pitch and roll information
 *
 * @param yprAttitude
 * 		yaw, pitch and roll
 *
 * @param yprRate
 * 		rate
 *
 * @param xyzAcc
 * 		acceleration
 *
 * @param xyzGravity
 * 		gravity
 *
 * @param xyzMagnet
 * 		magnet
 *
 * @return
 *		void
 *
 */
void getYawPitchRollInfo(float *yprAttitude, float *yprRate,
		float *xyzAcc, float *xComponent, float *yComponent, float *zComponent, float *xyzMagnet) {

	float q[4];		    // [w, x, y, z]         quaternion container
	float mXComponent[3];              
	float mYComponent[3];   
	float mZComponent[3];             
	float ax = 0.f;
	float ay = 0.f;
	float az = 0.f;
	float gx = 0.f;
	float gy = 0.f;
	float gz = 0.f;
#ifdef MPU6050_9AXIS
	short s_mx=0;
	short s_my=0;
	short s_mz=0;
	float f_mx=0.f;
	float f_my=0.f;
	float f_mz=0.f;
	float f_x=0.f;
	float f_y=0.f;
	float f_z=0.f;
#endif

	getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef MPU6050_9AXIS
	if(pollingMagnetDataBySingleMeasurementMode(&s_mx, &s_my, &s_mz)){
		
		f_x = (float)s_mx - mag_hard_iron_cal[0];
		f_y = (float)s_my - mag_hard_iron_cal[1];
		f_z = (float)s_mz - mag_hard_iron_cal[2];
		f_mx = f_x * mag_soft_iron_cal[0][0] + f_y * mag_soft_iron_cal[0][1] + f_z * mag_soft_iron_cal[0][2];
		f_my = f_x * mag_soft_iron_cal[1][0] + f_y * mag_soft_iron_cal[1][1] + f_z * mag_soft_iron_cal[1][2];
		f_mz = f_x * mag_soft_iron_cal[2][0] + f_y * mag_soft_iron_cal[2][1] + f_z * mag_soft_iron_cal[2][2];

		pushSmaData(&x_magnetSmaFilterEntry,f_mx);
		pushSmaData(&y_magnetSmaFilterEntry,f_my);
		pushSmaData(&z_magnetSmaFilterEntry,f_mz);
		f_mx = pullSmaData(&x_magnetSmaFilterEntry);
		f_my = pullSmaData(&y_magnetSmaFilterEntry);
		f_mz = pullSmaData(&z_magnetSmaFilterEntry);
	
		IMUupdate9(gx, gy, gz, ax, ay, az, f_my, f_mx, f_mz, q);
		
	}else{
		IMUupdate6(gx, gy, gz, ax, ay, az, q);
	}
	
#else
	IMUupdate6(gx, gy, gz, ax, ay, az, q);
#endif	

	GetXComponent(mXComponent, q);
	GetYComponent(mYComponent, q);
	GetZComponent(mZComponent, q);
	
	GetYawPitchRoll(yprAttitude, q, mZComponent);

	yprAttitude[0] = yprAttitude[0] * RA_TO_DE;
	yprAttitude[1] = yprAttitude[1] * RA_TO_DE;
	yprAttitude[2] = yprAttitude[2] * RA_TO_DE;
	xComponent[0] = mXComponent[0];
	xComponent[1] = mXComponent[1];
	xComponent[2] = mXComponent[2];
	yComponent[0] = mYComponent[0];
	yComponent[1] = mYComponent[1];
	yComponent[2] = mYComponent[2];
	zComponent[0] = mZComponent[0];
	zComponent[1] = mZComponent[1];
	zComponent[2] = mZComponent[2];
	yprRate[0] = gx * RA_TO_DE;
	yprRate[1] = gy * RA_TO_DE;
	yprRate[2] = gz * RA_TO_DE;
	xyzAcc[0] = ax;
	xyzAcc[1] = ay;
	xyzAcc[2] = az;

}

/**
 * get attitude
 *
 * @param data
 * 		output data
 *
 * @param q
 * 		quaternion
 *
 * @param gravity
 * 		gravity
 *
 * @return
 *		void
 *
 */
unsigned char GetYawPitchRoll(float *data, float *q, float *gravity) {
	// yaw: (about Z axis)
	data[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(
			gravity[0]
					/ sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(
			gravity[1]
					/ sqrt(gravity[0] * gravity[0] + gravity[2] * gravity[2]));
	return 0;
}

/**
 * get z component
 *
 * @param z
 * 		z component
 *
 * @param q
 * 		quaternion
 *
 * @return
 *		void
 *
 */
void GetZComponent(float *z, float *q) {
 
	z[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	z[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
	z[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

/**
 * get x component
 *
 * @param x
 * 		x component
 *
 * @param q
 * 		quaternion
 *
 * @return
 *		void
 *
 */
void GetXComponent(float *x, float *q) {

	x[0] = q[1] * q[1] + q[0] * q[0] - q[3] * q[3] - q[2] * q[2];
	x[1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	x[2] = 2 * (q[1] * q[3] + q[0] * q[2]);

}

/**
 * get y component
 *
 * @param y
 * 		y component
 *
 * @param q
 * 		quaternion
 *
 * @return
 *		void
 *
 */
void GetYComponent(float *y, float *q) {

	y[0] = 2 * (q[1] * q[2] + q[0] * q[3]);
	y[1] = q[2] * q[2] - q[3] * q[3] + q[0] * q[0] - q[1] * q[1];
	y[2] = 2 * (q[2] * q[3] - q[0] * q[1]);

}

/**
 * magnet calibration mode gets IMU Raw data
 *
 * @param 
 * 		void
 *
 *
 * @return
 *		void
 *
 */
void magnetCalibrationGetImuRawData(void){

	if(pollingMagnetDataBySingleMeasurementMode(&imuRawData[6], &imuRawData[7], &imuRawData[8])){

		getMotion6RawData(&imuRawData[0], &imuRawData[1], &imuRawData[2], &imuRawData[3], &imuRawData[4], &imuRawData[5]);	

	}
							
}

/**
 * get magnet calibration Raw data
 *
 * @param 
 * 		void
 *
 *
 * @return
 *		void
 *
 */
void getMagnetCalibrationRawData(short *rawData){

	rawData[0] = imuRawData[0];
	rawData[1] = imuRawData[1];
	rawData[2] = imuRawData[2];
	rawData[3] = imuRawData[3];
	rawData[4] = imuRawData[4];
	rawData[5] = imuRawData[5];
	rawData[6] = imuRawData[6];
	rawData[7] = imuRawData[7];
	rawData[8] = imuRawData[8];
	
}

/**
 * get magnet calibration data
 *
 * @param calCount
 * 		pointer for calibraction count
 *
 * @param hardIron
 * 		array for hard iron
 *
 * @param softIron
 * 		array for soft iron
 *
 *
 * @return bool
 *		parse Magnet CalibrationData successfully or not
 *
 */
bool parseMagnetCalibrationData(int *calCount, float *hardIron, float softIron[3][3]){
 
	char buf[1024];
	FILE *fptr;
	cJSON *pJsonRoot;
	cJSON *pSubJsonHardIron;
	cJSON *pSubJsonSoftIron;
	cJSON *pSub;
	
	fptr = fopen(MAGNET_CAL_DATA_PATH, "r");
	
	if (NULL == fptr)
	{
		_DEBUG(DEBUG_NORMAL,"MagnetCal.data doesn't exist\n");
		return false;
	}
	else
	{
		memset(buf,'\0',sizeof(buf));
		
		fread(buf, 1, sizeof(buf), fptr);
		fclose(fptr);
		
		pJsonRoot = cJSON_Parse(buf);
		if(NULL == pJsonRoot){
		   return false;
		}
		
		pSub = cJSON_GetObjectItem(pJsonRoot, "Calibration Count");
		*calCount = pSub->valueint;
		
		pSubJsonHardIron = cJSON_GetObjectItem(pJsonRoot, "Hard Iron");
		if(NULL == pSubJsonHardIron){
		   return false;
}
		pSub = cJSON_GetObjectItem(pSubJsonHardIron, "0");
		hardIron[0] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonHardIron, "1");
		hardIron[1] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonHardIron, "2");
		hardIron[2] = pSub->valuedouble;
		
		pSubJsonSoftIron = cJSON_GetObjectItem(pJsonRoot, "Soft Iron");
		if(NULL == pSubJsonSoftIron){
		   return false;
		}
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "00");
		softIron[0][0] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "01");
		softIron[0][1] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "02");
		softIron[0][2] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "10");
		softIron[1][0] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "11");
		softIron[1][1] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "12");
		softIron[1][2] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "20");
		softIron[2][0] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "21");
		softIron[2][1] = pSub->valuedouble;
		pSub = cJSON_GetObjectItem(pSubJsonSoftIron, "22");
		softIron[2][2] = pSub->valuedouble;

		
		cJSON_Delete(pJsonRoot);
	}
	
	return true;

  }

