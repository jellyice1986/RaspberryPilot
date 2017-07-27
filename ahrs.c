/******************************************************************************
The ahrs.c in RaspberryPilot project is placed under the MIT license

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

=============================================================

ATTENTION:

The code in this file is mostly copied and rewritten from

X-IO Technologies:
http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include "commonLib.h"
#include "mpu6050.h"
#include "ahrs.h"

#if defined(MADGWICK_AHRS)
#define beta 0.5f
#elif defined(MAHONY_AHRS)
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.05f)	// 2 * integral gain
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
#endif

static struct timeval last_tv;
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

/**
 * init ahrs
 *
 * @param
 * 		void
 *
 * @return
 *		void
 *
 */
void ahrsInit() {
	q0 = 1;
	q1 = 0;
	q2 = 0;
	q3 = 0;
}

/**
 * fast inverse square root
 *
 * @param x
 * 		input value
 *
 * @return
 *		inverse square
 *
 */
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**
 * Madgwick's IMU update method
 *
 * reference:
 * S. O. H. Madgwick, An efficient orientation filter for inertial and inertial/magnetic sensor arrays, Technical report, University of. Bristol University, UK, 2010
 *
 * @param gx
 * 		Gyroscope x axis measurement in radians/s
 *
 * @param gy
 * 		Gyroscope y axis measurement in radians/s
 *
 * @param gz
 * 		Gyroscope z axis measurement in radians/s
 *
 * @param ax
 * 		Accelerometer x axis measurement in any calibrated units
 *
 * @param ay
 * 		Accelerometer y axis measurement in any calibrated units
 *
 * @param az
 * 		Accelerometer z axis measurement in any calibrated units
 *
 * @param q
 * 		quaternion
 *
 * @return
 *		void
 *
 */
void IMUupdate6(float gx, float gy, float gz, float ax, float ay, float az,
		float q[]) {

#if defined(MADGWICK_AHRS)

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float timeDiff = 0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (TIME_IS_UPDATED(last_tv)) {
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
			timeDiff = GET_SEC_TIMEDIFF(tv,last_tv);
			
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_4q0 = 4.0f * q0;
			_4q1 = 4.0f * q1;
			_4q2 = 4.0f * q2;
			_8q1 = 8.0f * q1;
			_8q2 = 8.0f * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

			// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (timeDiff);
		q1 += qDot2 * (timeDiff);
		q2 += qDot3 * (timeDiff);
		q3 += qDot4 * (timeDiff);

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
	}

	UPDATE_LAST_TIME(tv,last_tv);

#elif defined(MAHONY_AHRS)

	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float timeDiff = 0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (TIME_IS_UPDATED(last_tv)) {

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			timeDiff = GET_SEC_TIMEDIFF(tv,last_tv);
			
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;		
	
			// Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = q1 * q3 - q0 * q2;
			halfvy = q0 * q1 + q2 * q3;
			halfvz = q0 * q0 - 0.5f + q3 * q3;
		
			// Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy);
			halfey = (az * halfvx - ax * halfvz);
			halfez = (ax * halfvy - ay * halfvx);
	
			// Compute and apply integral feedback if enabled
			if(twoKiDef > 0.0f) {
				integralFBx += twoKiDef * halfex * (timeDiff);	// integral error scaled by Ki
				integralFBy += twoKiDef * halfey * (timeDiff);
				integralFBz += twoKiDef * halfez * (timeDiff);
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
			else {
				integralFBx = 0.0f; // prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}
	
			// Apply proportional feedback
			gx += twoKpDef * halfex;
			gy += twoKpDef * halfey;
			gz += twoKpDef * halfez;
		}
		
		// Integrate rate of change of quaternion
		gx *= (0.5f * timeDiff); 	// pre-multiply common factors
		gy *= (0.5f * timeDiff);
		gz *= (0.5f * timeDiff);
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx); 
		
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;

	}

	UPDATE_LAST_TIME(tv,last_tv);
	
#endif
}


/**
 * Madgwick's IMU update method
 *
 * reference:
 * S. O. H. Madgwick, An efficient orientation filter for inertial and inertial/magnetic sensor arrays, Technical report, University of. Bristol University, UK, 2010
 *
 * @param gx
 * 		Gyroscope x axis measurement in radians/s
 *
 * @param gy
 * 		Gyroscope y axis measurement in radians/s
 *
 * @param gz
 * 		Gyroscope z axis measurement in radians/s
 *
 * @param ax
 * 		Accelerometer x axis measurement in any calibrated units
 *
 * @param ay
 * 		Accelerometer y axis measurement in any calibrated units
 *
 * @param az
 * 		Accelerometer z axis measurement in any calibrated units
 *
 * @param q
 * 		quaternion
 *
 * @return
 *		void
 *
 */
void IMUupdate9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
		float q[]) {

#if defined(MADGWICK_AHRS)

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float timeDiff=0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (TIME_IS_UPDATED(last_tv)) {
		
		timeDiff = GET_SEC_TIMEDIFF(tv,last_tv);

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0f * (timeDiff));
		q1 += qDot2 * (1.0f * (timeDiff));
		q2 += qDot3 * (1.0f * (timeDiff));
		q3 += qDot4 * (1.0f * (timeDiff));

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		q[0]=q0;
		q[1]=q1;
		q[2]=q2;
		q[3]=q3;

	}

	UPDATE_LAST_TIME(tv,last_tv);

#elif defined(MAHONY_AHRS)

	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float timeDiff=0.f;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (TIME_IS_UPDATED(last_tv)) {
		
		timeDiff = GET_SEC_TIMEDIFF(tv,last_tv);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;     

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;   

	        // Auxiliary variables to avoid repeated arithmetic
	        q0q0 = q0 * q0;
	        q0q1 = q0 * q1;
	        q0q2 = q0 * q2;
	        q0q3 = q0 * q3;
	        q1q1 = q1 * q1;
	        q1q2 = q1 * q2;
	        q1q3 = q1 * q3;
	        q2q2 = q2 * q2;
	        q2q3 = q2 * q3;
	        q3q3 = q3 * q3;   

	        // Reference direction of Earth's magnetic field
	        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	        bx = sqrt(hx * hx + hy * hy);
	        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;
	        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
	        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
	        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
		
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
			halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
			halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

			// Compute and apply integral feedback if enabled
			if(twoKiDef > 0.0f) {
				integralFBx += twoKiDef * halfex * timeDiff;	// integral error scaled by Ki
				integralFBy += twoKiDef * halfey * timeDiff;
				integralFBz += twoKiDef * halfez * timeDiff;
				
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
			else {
				integralFBx = 0.0f;	// prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKpDef * halfex;
			gy += twoKpDef * halfey;
			gz += twoKpDef * halfez;
		}
		
		// Integrate rate of change of quaternion
		//gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
		//gy *= (0.5f * (1.0f / sampleFreq));
		//gz *= (0.5f * (1.0f / sampleFreq));

		gx *= (0.5f * timeDiff);		// pre-multiply common factors
		gy *= (0.5f * timeDiff);
		gz *= (0.5f * timeDiff);

		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx); 
		
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		q[0]=q0;
		q[1]=q1;
		q[2]=q2;
		q[3]=q3;

	}

	UPDATE_LAST_TIME(tv,last_tv);
	
#endif
}

