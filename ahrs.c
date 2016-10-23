
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "commonLib.h"
#include "mpu6050.h"
#include "ahrs.h"

#define Kp 2.5f                     // proportional gain governs rate of convergence to accelerometer/magnetometer

static struct timeval last_tv;
static float exInt = 0, eyInt = 0, ezInt = 0;
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

float invSqrt(float x) ;

void ahrsInit() {
	q0 = 1; 
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,
		float q[]) {

	float norm = 0.f;
	float vx = 0.f, vy = 0.f, vz = 0.f;
	float ex = 0.f, ey = 0.f, ez = 0.f;
	float timeDiff = 0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (last_tv.tv_sec != 0) {

		timeDiff = (float) ((float) (tv.tv_sec - last_tv.tv_sec)
				+ (float) (tv.tv_usec - last_tv.tv_usec)*0.000001f);

		_DEBUG(DEBUG_IMUUPDATE_INTVAL,"(%s-%d) interval=%.5f Sec\n",__func__,__LINE__,timeDiff);

		// normalise the measurements
		norm = invSqrt(ax * ax + ay * ay + az * az);
		ax = ax * norm;
		ay = ay * norm;
		az = az * norm;

		// estimated direction of gravity
		vx = 2 * (q1 * q3 - q0 * q2);
		vy = 2 * (q0 * q1 + q2 * q3);
		vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		// error is sum of cross product between reference direction of field and direction measured by sensor
		ex = (ay * vz - az * vy);
		ey = (az * vx - ax * vz);
		ez = (ax * vy - ay * vx);

		// integral error scaled integral gain
		exInt = exInt + (ex * timeDiff);
		eyInt = eyInt + (ey * timeDiff);
		ezInt = ezInt + (ez * timeDiff);

		// adjusted gyroscope measurements
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;

		// integrate quaternion rate and normalise
		q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * timeDiff * 0.5f;
		q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * timeDiff * 0.5f;
		q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * timeDiff * 0.5f;
		q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * timeDiff * 0.5f;

		// normalise quaternion
		norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = q0 * norm;
		q1 = q1 * norm;
		q2 = q2 * norm;
		q3 = q3 * norm;
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;

	}

	last_tv.tv_usec = tv.tv_usec;
	last_tv.tv_sec = tv.tv_sec;

}

