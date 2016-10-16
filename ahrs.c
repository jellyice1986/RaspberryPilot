#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mpu6050.h"
#include "ahrs.h"

#define Kp 2.5f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.004f                // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.002f                // half the sample period        : 0.005s/2=0.0025s
#define INIT_SAMPLE_COUNTER 100.f

static struct timeval last_tv;
static float exInt = 0, eyInt = 0, ezInt = 0;
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

void initQuaternion();
float invSqrt(float x) ;


void ahrsInit() {
	//initQuaternion();
}

void initQuaternion() {
#if 0
	float a_init_ax = 0.f;
	float a_init_ay = 0.f;
	float a_init_az = 0.f;
	float a_init_gx = 0.f;
	float a_init_gy = 0.f;
	float a_init_gz = 0.f;
	float a_init_mx = 0.f;
	float a_init_my = 0.f;
	float a_init_mz = 0.f;
	float init_ax = 0;
	float init_ay = 0;
	float init_az = 0;
	float init_gx = 0;
	float init_gy = 0;
	float init_gz = 0;
	float init_mx = 0;
	float init_my = 0;
	float init_mz = 0;
	float init_Roll = 0.f;
	float init_Pitch = 0.f;
	float init_Yaw = 0.f;
	int i = 0;

	for (i = 0; i < INIT_SAMPLE_COUNTER; i++) {
		getMotion9(&a_init_ax, &a_init_ay, &a_init_az, &a_init_gx, &a_init_gy,
				&a_init_gz, &a_init_mx, &a_init_my, &a_init_mz);
		init_ax = init_ax + a_init_ax;
		init_ay = init_ay + a_init_ay;
		init_az = init_az + a_init_az;
		init_gx = init_gx + a_init_gx;
		init_gy = init_gy + a_init_gz;
		init_gz = init_gz + a_init_gz;
		init_mx = init_mx + a_init_mx;
		init_my = init_my + a_init_my;
		init_mz = init_mz + a_init_mz;
		usleep(5000);
	}

	init_ax = init_ax / INIT_SAMPLE_COUNTER;
	init_ay = init_ay / INIT_SAMPLE_COUNTER;
	init_az = init_az / INIT_SAMPLE_COUNTER;
	init_gx = init_gx / INIT_SAMPLE_COUNTER;
	init_gy = init_gy / INIT_SAMPLE_COUNTER;
	init_gz = init_gz / INIT_SAMPLE_COUNTER;
	init_mx = init_mx / INIT_SAMPLE_COUNTER;
	init_my = init_my / INIT_SAMPLE_COUNTER;
	init_mz = init_mz / INIT_SAMPLE_COUNTER;

	init_Roll = atan2(init_ay, init_az);
	init_Pitch = -asin(init_ax);			  //init_Pitch = asin(ay / 1);		
	init_Yaw = -atan2(
			init_my * cos(init_Roll)
					+ init_mx * sin(init_Roll) * sin(init_Pitch)
					- init_mz * sin(init_Roll) * cos(init_Pitch),
			init_mx * cos(init_Pitch) + init_mz * sin(init_Pitch));	//atan2(my, mx);
	q0 = cos(0.5 * init_Roll) * cos(0.5 * init_Pitch) * cos(0.5 * init_Yaw)
			+ sin(0.5 * init_Roll) * sin(0.5 * init_Pitch)
					* sin(0.5 * init_Yaw);  //w
	q1 = sin(0.5 * init_Roll) * cos(0.5 * init_Pitch) * cos(0.5 * init_Yaw)
			- cos(0.5 * init_Roll) * sin(0.5 * init_Pitch)
					* sin(0.5 * init_Yaw);  //x
	q2 = cos(0.5 * init_Roll) * sin(0.5 * init_Pitch) * cos(0.5 * init_Yaw)
			+ sin(0.5 * init_Roll) * cos(0.5 * init_Pitch)
					* sin(0.5 * init_Yaw);  //y
	q3 = cos(0.5 * init_Roll) * cos(0.5 * init_Pitch) * sin(0.5 * init_Yaw)
			- sin(0.5 * init_Roll) * sin(0.5 * init_Pitch)
					* cos(0.5 * init_Yaw);  //z

#endif
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

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, float q[]) {

#if 1
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float wx, wy, wz;
	float hx, hy, hz, bx, bz;
	float timeDiff = 0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (last_tv.tv_sec != 0) {

		timeDiff = (float) ((float) (tv.tv_sec - last_tv.tv_sec)
				+ (float) (tv.tv_usec - last_tv.tv_usec) / 1000000.f);

		// normalise the measurements
		norm = sqrt(ax * ax + ay * ay + az * az);
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		// estimated direction of gravity (v)
		vx = 2 * (q1 * q3 - q0 * q2);
		vy = 2 * (q0 * q1 + q2 * q3);
		vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		// estimated direction of flux (w)
		norm = sqrt(mx * mx + my * my + mz * mz);
		mx = mx / norm;
		my = my / norm;
		mz = mz / norm;
#if 0
		// compute reference direction of flux
		hx = 2 * mx * (0.5 - q2 * q2 - q3 * q3) + 2 * my * (q1 * q2 - q0 * q3)
		+ 2 * mz * (q1 * q3 + q0 * q2);
		hy = 2 * mx * (q1 * q2 + q0 * q3) + 2 * my * (0.5 - q1 * q1 - q3 * q3)
		+ 2 * mz * (q2 * q3 - q0 * q1);
		hz = 2 * mx * (q1 * q3 - q0 * q2) + 2 * my * (q2 * q3 + q0 * q1)
		+ 2 * mz * (0.5 - q1 * q1 - q2 * q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = hz;

		wx = 2 * bx * (0.5 - q2 * q2 - q3 * q3) + 2 * bz * (q1 * q3 - q0 * q2);
		wy = 2 * bx * (q1 * q2 - q0 * q3) + 2 * bz * (q0 * q1 + q2 * q3);
		wz = 2 * bx * (q0 * q2 + q1 * q3) + 2 * bz * (0.5 - q1 * q1 - q2 * q2);
#else
		// Reference direction of Earth's magnetic field
		hx = 2 * mx * (0.5 - q2 * q2 - q3 * q3) + 2 * my * (q1 * q2 - q0 * q3)
				+ 2 * mz * (q1 * q3 + q0 * q2);
		hy = 2 * mx * (q1 * q2 + q0 * q3) + 2 * my * (0.5 - q1 * q1 - q3 * q3)
				+ 2 * mz * (q2 * q3 - q0 * q1);
		hz = 2 * mx * (q1 * q3 - q0 * q2) + 2 * my * (q2 * q3 + q0 * q1)
				+ 2 * mz * (0.5 - q1 * q1 - q2 * q2);
		bx = sqrt((hx * hx) + (hy * hy));
		//by = sqrtf((hx*hx) + (hy*hy));
		bz = hz;

		// Estimated direction of magnetic field
		wx = 2 * bx * (0.5 - q2 * q2 - q3 * q3) + 2 * bz * (q1 * q3 - q0 * q2);
		wy = 2 * bx * (q1 * q2 - q0 * q3) + 2 * bz * (q0 * q1 + q2 * q3);
		wz = 2 * bx * (q0 * q2 + q1 * q3) + 2 * bz * (0.5 - q1 * q1 - q2 * q2);
		//		  wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
		//		  wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
		//		  wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);

#endif
		// error is sum of cross product between reference direction of fields and direction measured by sensors
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

		// integral error scaled integral gain
		exInt = exInt + ex * timeDiff;
		eyInt = eyInt + ey * timeDiff;
		ezInt = ezInt + ez * timeDiff;

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
		norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm;
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
	}
	last_tv.tv_usec = tv.tv_usec;
	last_tv.tv_sec = tv.tv_sec;
	//printf("q0=%3.3f, q1=%3.3f, q2=%3.3f, q3=%3.3f\n",q0,q1,q2,q3);
//	printf("q0=%3.3f, q1=%3.3f, q2=%3.3f, q3=%3.3f\n",q0,q1,q2,q3);
#endif
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
				+ (float) (tv.tv_usec - last_tv.tv_usec) / 1000000.f);

		//printf("haltTime=%3.7f s\n",timeDiff);
		//printf("haltTime*0.5=%3.3f s\n",timeDiff*0.5f);

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
	//printf("q0=%3.3f, q1=%3.3f, q2=%3.3f, q3=%3.3f\n",q0,q1,q2,q3);

}

