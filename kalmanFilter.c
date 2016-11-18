/******************************************************************************
The kalmanFilter.c in RaspberryPilot project is placed under the MIT license

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

#include <string.h>
#include "kalmanFilter.h"

/**
 * 1 dimension Kalman filter
 *
 * @param inData
 * 		measurement data
 *
 * @param kalmanStruct
 * 		 a Kalman filter entity
 *
 * @return
 *		output
 *
 */
float kalmanFilterOneDimCalc(float inData,KALMAN_1D_STRUCT *kalmanStruct){                        

    kalmanStruct->p = kalmanStruct->p+kalmanStruct->q;
    kalmanStruct->kGain = kalmanStruct->p/(kalmanStruct->p+kalmanStruct->r);
    inData = kalmanStruct->prevData+(kalmanStruct->kGain*(inData-kalmanStruct->prevData));
    kalmanStruct->p = (1-kalmanStruct->kGain)*kalmanStruct->p;
	kalmanStruct->prevData = inData;

    return inData;
}

/**
 * init a  1 dimension Kalman filter entity
 *
 * @param kalmanStruct
 * 		 a Kalman filter entity
 *
 * @param name
 * 		 name of this entity
 *
 * @param prevData
 * 		 previous data
 *
 * @param p
 * 		 estimate covariance
 *
 * @param q
 * 		 covariance of process noise
 *
 * @param r
 * 		 covariance of observation noise
 *
 * @param kGain
 * 		 Kalman gain
 *
 * @return
 *		void
 *
 */
void initkalmanFilterOneDimEntity(KALMAN_1D_STRUCT *kalmanStruct,char *name, float prevData, float p,float q,float r, float kGain){
	
	strcpy(kalmanStruct->name, name);
	kalmanStruct->prevData=prevData;
    kalmanStruct->p=p;   
	kalmanStruct->q=q; 
	kalmanStruct->r=r;   
	kalmanStruct->kGain=kGain; 
}