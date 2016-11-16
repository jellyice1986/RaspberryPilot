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
#include <unistd.h>
#include "commonLib.h"
#include "i2c.h"

#define SRF02_ADD_R    		0xE1
#define SRF02_ADD_W    		0xE0
#define SRF02_REG_CMD       0x00
#define SRF02_REG_RANGE_H   0x02
#define SRF02_REG_RANGE_L   0x03
#define SRF02_CMD_INCH    	0x50
#define SRF02_CMD_CM      	0x51
#define SRF02_CMD_US      	0x52

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

	bool result=true;
	
	result=checkI2cDeviceIsExist(SRF02_ADD_R);
	result=checkI2cDeviceIsExist(SRF02_ADD_W);

	return result;
	
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

	result=writeByte(SRF02_ADD_W,SRF02_REG_CMD,SRF02_CMD_CM);
	if(!result) return false;

	usleep(70000);

	result=(readBytes(SRF02_ADD_R,SRF02_REG_RANGE_H,2,data)<0) ? false:true;
	if(!result) return false;
		
	*cm=((data[0] << 8) | data[1]);

	usleep(500);

	return true;
}


