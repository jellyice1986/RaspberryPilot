#include <stdio.h>
#include <memory.h>

#include "smoother.h"


/**
 * format of smoother buffer :
 * buffer[0]= current sample index
 * buffer[n]=data, 1 < n < SMOOTHER_BUFFER_SIZE
 */
float roll_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
float pitch_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
float yaw_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
float roll_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
float pitch_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
float yaw_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];

/**
 *  init smoother buffer
 */
void initSmootherBuffer() {
	memset(roll_gyro_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	memset(pitch_gyro_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	memset(yaw_gyro_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	memset(roll_attitude_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	memset(pitch_attitude_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	memset(yaw_attitude_smoother_buffer, 0, SMOOTHER_BUFFER_SIZE + 1);
	roll_gyro_smoother_buffer[0]=1;
	pitch_gyro_smoother_buffer[0]=1;
	yaw_gyro_smoother_buffer[0]=1;
    roll_attitude_smoother_buffer[0]=1;
    pitch_attitude_smoother_buffer[0]=1;
    yaw_attitude_smoother_buffer[0]=1;

}

/**
 *  Get data from smoother
 *  @param buf
 *  			buffer of smoother
 */
float getDataFromSmoother(float *buf) {

	int i = 0;
	float averageValue = 0;

	for (i = 1; i < (SMOOTHER_BUFFER_SIZE + 1); i++) {
		averageValue = averageValue + buf[i];
	}
	return (averageValue / (float) SMOOTHER_BUFFER_SIZE);
}

/**
 *  put data into smoother
 *  @param buf
 *  			buffer of smoother
 *
 *  @param value
 *  			sample value
 */
void addDataToSmoother(float *buf, float value) {

	if ( (int)buf[0] >= SMOOTHER_BUFFER_SIZE + 1)
		buf[0] = 1.0;

	//printf("test buf[0]=%d\n",(int)buf[0]);
	buf[(int)buf[0]] = value;
	buf[0]++;
}
