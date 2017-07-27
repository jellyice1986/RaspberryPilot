/******************************************************************************
The srf02.c in RaspberryPilot project is placed under the MIT license

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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "commonLib.h"
#include "kalmanFilter.h"
#include "i2c.h"

#define SRF02_ADD   		0x70
#define SRF02_REG_CMD       0x00
#define SRF02_REG_RANGE_H   0x02
#define SRF02_CMD_CM      	0x51

static KALMAN_1D_STRUCT srf02KalmanFilterEntry;


/**
 * init SRF02
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool srf02Init(){
	
	if (checkI2cDeviceIsExist(SRF02_ADD)) {
		_DEBUG(DEBUG_NORMAL, "(%s-%d) SRF02 exist\n", __func__, __LINE__);
	} else {
		_ERROR("(%s-%d) SRF02 dowsn't exist\n", __func__, __LINE__);
		return false;
	}

	initkalmanFilterOneDimEntity(&srf02KalmanFilterEntry,"SRF02", 0.f,10.f,1.f,5.f, 0.f);

	return true;
	
}

/**
 * get measurement data from SRF02
 *
 * @param
 * 		data
 *
 * @return
 *		bool
 *
 */
bool srf02GetMeasurementData(unsigned short *cm){


	bool result=true;
	unsigned char data[2];

	result=writeByte(SRF02_ADD,SRF02_REG_CMD,SRF02_CMD_CM);
	if(!result) return false;

	usleep(500);

	result=(readBytes(SRF02_ADD,SRF02_REG_RANGE_H,2,data)<0) ? false:true;
	if(!result) return false;
		
	*cm =(unsigned short)kalmanFilterOneDimCalc(((data[0] << 8) | data[1]) ,&srf02KalmanFilterEntry);

	usleep(500);

	return true;
}


