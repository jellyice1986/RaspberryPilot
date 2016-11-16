/******************************************************************************
The pca9685.c in RaspberryPilot project is placed under the MIT license

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

#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>     
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include "commonLib.h"
#include "i2c.h"
#include "pca9685.h"

#define PCA9685_ADDRESS 		0x40			//PCA9685 i2c address
#define PCA9685_MODE1 			0x00			//Mode  register  1
#define PCA9685_MODE2 			0x01			//Mode  register  2
#define PCA9685_LED0_ON_L 		0x6				//LED0 output and brightness control byte 0
#define PCA9685_LED0_ON_H 		0x7				//LED0 output and brightness control byte 1
#define PCA9685_LED0_OFF_L 		0x8				//LED0 output and brightness control byte 2
#define PCA9685_LED0_OFF_H 		0x9				//LED0 output and brightness control byte 3
#define PCA9685_LED_MULTIPLYER 	4				// For the other 15 channels
#define PCA9685_PRE_SCALE 		0xFE			//prescaler for output frequency
#define PCA9685_CLOCK_FREQ 		25000000.f 		//25MHz default osc clock

static bool PCA9685_initSuccess = false;

/**
 * Init PCA9685
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool pca9685Init() {

	if (checkI2cDeviceIsExist(PCA9685_ADDRESS)) {
		_DEBUG(DEBUG_NORMAL, "PCA9685 exist\n");
	} else {
		_DEBUG(DEBUG_NORMAL, "PCA9685_ADDRESS dowsn't exist\n");
		return false;
	}

	PCA9685_initSuccess = true;
	resetPca9685();
	return true;
}

/**
 * Reset PCA9685
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
void resetPca9685() {

	if (true == PCA9685_initSuccess) {
		writeByte(PCA9685_ADDRESS, PCA9685_MODE1, 0x00); //setup sleep mode, Low power mode. Oscillator off (bit4: 1-sleep, 0-normal)
		writeByte(PCA9685_ADDRESS, PCA9685_MODE2, 0x04);
		//Delay Time is 0, means it always turn into high at the begin
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_L + PCA9685_LED_MULTIPLYER * 0, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_H + PCA9685_LED_MULTIPLYER * 0, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_L + PCA9685_LED_MULTIPLYER * 1, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_H + PCA9685_LED_MULTIPLYER * 1, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_L + PCA9685_LED_MULTIPLYER * 2, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_H + PCA9685_LED_MULTIPLYER * 2, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_L + PCA9685_LED_MULTIPLYER * 3, 0);
		writeByte(PCA9685_ADDRESS,
		PCA9685_LED0_ON_H + PCA9685_LED_MULTIPLYER * 3, 0);

	} else {
		_ERROR("(%s-%d) pca9685 doesn't init\n", __func__, __LINE__);
	}
}

/**
 * Set the frequency of PWM
 *
 * @param freq
 * 		frequency of PWM
 *
 * @return
 *		bool
 */
void setPWMFreq(unsigned short freq) {

	_DEBUG(DEBUG_NORMAL, "(%s-%d)PCA9685: setting frequency %d HZ\n", __func__,
			__LINE__, freq);

	unsigned char prescale = (PCA9685_CLOCK_FREQ / 4096 / freq) - 1;
	unsigned char oldmode = 0;
	readByte(PCA9685_ADDRESS, PCA9685_MODE1, &oldmode);

	unsigned char newmode = (oldmode & 0x7F) | 0x10; //setup sleep mode, Low power mode. Oscillator off (bit4: 1-sleep, 0-normal)

	writeByte(PCA9685_ADDRESS, PCA9685_MODE1, newmode);
	writeByte(PCA9685_ADDRESS, PCA9685_PRE_SCALE, prescale); //set freq
	writeByte(PCA9685_ADDRESS, PCA9685_MODE1, oldmode); //setup normal mode (bit4: 1-sleep, 0-normal)
	usleep(1000); // >500us
	writeByte(PCA9685_ADDRESS, PCA9685_MODE1, oldmode | 0x80); //setup restart (bit7: 1- enable, 0-disable)
	usleep(1000); // >500us
}

/**
 * set PWM signal
 *    
 * =======================================
 *
 *|<------------- 0 to 4095 ------------->|
 *
 *  	       	---------------------
 *  Low      	| High                          |  Low
 *  --------------------------------------
 * ^                ^                               ^
 * Delay Time  On Time                     Off Time
 *
 * ========================================
 *
 *   Example:
 *   If on Time=0, off Time=2014 then the PWM signal is as below
 *
 * 	--------------------
 *    | High                         |  Low
 * 	-------------------------------------
 * 	^      		             ^
 *  0 (On Time)                 2014 (Off Time)
 *
 * @param channel
 * 		channel index
 *
 * @param value
 * 		PWM value from 0 to 4095
 */
void setPWM(unsigned char channel, unsigned short value) {

	if (!PCA9685_initSuccess) {
		_ERROR("(%s-%d)  PCA9685_initSuccess=%d\n", __func__, __LINE__,
				PCA9685_initSuccess);
		return;
	}

	writeByte(PCA9685_ADDRESS,
 		PCA9685_LED0_OFF_L + PCA9685_LED_MULTIPLYER * channel, value & 0xFF);
 	writeByte(PCA9685_ADDRESS,
 		PCA9685_LED0_OFF_H + PCA9685_LED_MULTIPLYER * channel, value >> 8);	
	
}

