#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include "i2c.h"
#include "pca9685.h"

#define MODE1 0x00			//Mode  register  1
#define MODE2 0x01			//Mode  register  2
#define SUBADR1 0x02		//I2C-bus subaddress 1
#define SUBADR2 0x03		//I2C-bus subaddress 2
#define SUBADR3 0x04		//I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock


static bool PCA9685_initSuccess;


/**
 * Init PCA9685
 *
 */
void pca9685Init() {
	PCA9685_initSuccess = false;
	if(true== testPca9685Connection()){
		printf("PCA9685 is connected...\n");
		PCA9685_initSuccess = true;
		resetPca9685();
	}
}

/**
 * Reset PCA9685
 */
void resetPca9685() {
	if(true==PCA9685_initSuccess){
		writeByte(PCA9685_ADDRESS, MODE1, 0x00);//setup sleep mode, Low power mode. Oscillator off (bit4: 1-sleep, 0-normal)
		writeByte(PCA9685_ADDRESS, MODE2, 0x04);
		//Delay Time is 0, means it always turn into high at the begin
		writeByte(PCA9685_ADDRESS, LED0_ON_L + LED_MULTIPLYER * 0, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_H + LED_MULTIPLYER * 0, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_L + LED_MULTIPLYER * 1, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_H + LED_MULTIPLYER * 1, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_L + LED_MULTIPLYER * 2, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_H + LED_MULTIPLYER * 2, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_L + LED_MULTIPLYER * 3, 0);
		writeByte(PCA9685_ADDRESS, LED0_ON_H + LED_MULTIPLYER * 3, 0);


	}else{
		printf("pca9685 doesn't init\n");
	}
}

/**
 * Test  Connection of PCA9685
 * @return bool
 * 		whether PCA9685 is connected or not 
 */
bool testPca9685Connection() {
    char data;
    char status = readByte(PCA9685_ADDRESS, PRE_SCALE, &data);
    if (status > 0)
        return true;
    else
        return false;
}


/**
 * Set the frequency of PWM
 *
 * @param freq
 * 				 frequency of PWM
 *
 */
void setPWMFreq(unsigned short freq) {

	printf("PCA9685: setting frequency %d HZ\n",freq);
	unsigned char prescale = (CLOCK_FREQ / 4096 / freq) - 1;
	unsigned char oldmode = 0;
	readByte(PCA9685_ADDRESS, MODE1, &oldmode);
	printf("PCA9685: old oled mode: %d\n",oldmode);
	unsigned char newmode = (oldmode & 0x7F) | 0x10; //setup sleep mode, Low power mode. Oscillator off (bit4: 1-sleep, 0-normal)

	writeByte(PCA9685_ADDRESS, MODE1, newmode);
	writeByte(PCA9685_ADDRESS, PRE_SCALE, prescale); //set freq
	writeByte(PCA9685_ADDRESS, MODE1, oldmode);//setup normal mode (bit4: 1-sleep, 0-normal)
	usleep(1000); // >500us
	writeByte(PCA9685_ADDRESS, MODE1, oldmode | 0x80); //setup restart (bit7: 1- enable, 0-disable)
	usleep(1000); // >500us
}

/**
 * set PWM signal
 *    
 * =======================================
 *
 *	|<----------- 0 to 4095 ------------>|
 *
 *  	       		--------------------
 *    Low      		| High                         |  Low
 *  	-------------------------------------
 * 	^      		^                               ^
 *    Delay Time    	On Time (16 bits)       Off Time (16 bits)
 *
 * ========================================
 *
 *   Example:
 *   If on Time=0, off Time=2014 then the PWM signal is as below
 *
 *  	--------------------
 *    | High                         |  Low
 *  	-------------------------------------
 * 	^      		             ^
 *    0 (On Time)       	      2014 (Off Time) 
 *
 * @param channel
 * 				channel index
 *
 * @param value
 * 				PWM value from 0 to 4095
 */
void setPWM(unsigned char channel, unsigned short value) {

	//printf("channel = %d, value = %d\n",channel,value);
	
	if (0 == PCA9685_initSuccess) {
		printf("%s: PCA9685_initSuccess=%d\n", __func__, PCA9685_initSuccess);
		return;
	}
	
	//writeByte(PCA9685_ADDRESS, LED0_ON_L + LED_MULTIPLYER * channel, 0);
	//writeByte(PCA9685_ADDRESS, LED0_ON_H + LED_MULTIPLYER * channel, 0);
	writeByte(PCA9685_ADDRESS, LED0_OFF_L + LED_MULTIPLYER * channel, value & 0xFF);
	writeByte(PCA9685_ADDRESS, LED0_OFF_H + LED_MULTIPLYER * channel, value >> 8);
}

