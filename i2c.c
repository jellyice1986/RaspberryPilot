/******************************************************************************
The i2c.c in RaspberryPilot project is placed under the MIT license

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

The code in this file is mostly copied from

Jeff Rowberg:
https://github.com/jrowberg/i2cdevlib

Richard Hirst:
https://github.com/richardghirst/PiBits/blob/master/MPU6050-Pi-Demo

******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <memory.h>
#include <errno.h>
#include "commonLib.h"
#include "i2c.h"

/**
 * check whather a I2C devide is existing or not
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @return
 *		bool
 *
 */
bool checkI2cDeviceIsExist(unsigned char devAddr) {

	int fd = -1;
	bool result = true;
	unsigned char regAddr = 0x01;

	fd = open(I2C_DEV_PATH, O_RDWR);
	if (fd < 0) {
		result = false;
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		result = false;
	}
	if (write(fd, &regAddr, 1) != 1) {
		result = false;
	}

	close(fd);
	return result;
}

/**
 * write a byte into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param data
 * 		one bytes
 *
 * @return
 *		success or failure
 *
 */
bool writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data) {

	return writeBytes(devAddr, regAddr, 1, &data);
}

/**
 * write a bit into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param bitNum
 * 		the poisioion of the bit
 *
 * @param data
 * 		one bit
 *
 * @return
 *		success or failure
 *
 */
bool writeBit(unsigned char devAddr, unsigned char regAddr,

unsigned char bitNum, unsigned char data) {

	unsigned char mByte = 0x00;

	readByte(devAddr, regAddr, &mByte);
	mByte = (data != 0) ? (mByte | (1 << bitNum)) : (mByte & ~(1 << bitNum));

	return writeByte(devAddr, regAddr, mByte);
}

/**
 * write serveral bits into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param bitStart
 * 		the posioion of the start
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		serveral bits
 *
 * @return
 *		success or failure
 *
 */
bool writeBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char data) {

	unsigned char b;

	if (readByte(devAddr, regAddr, &b) != 0) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1);
		data &= mask;
		b &= ~(mask);
		b |= data;
		return writeByte(devAddr, regAddr, b);
	} else {
		return false;
	}
}

/**
 * write serveral bytes into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		serveral bits
 *
 * @return
 *		success or failure
 *
 */
bool writeBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char * data) {

	char count = 0;
	unsigned char buf[128];
	int fd;
	bool result=true;

	if (length > 127) {
		_ERROR("length (%d) > 127\n", length);
		result= false;
	}

	fd = open(I2C_DEV_PATH, O_RDWR);
	if (fd < 0) {
		_ERROR("%s: Failed to open device\n", __func__);
		result= false;
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		_ERROR("%s: Failed to select device\n", __func__);
		close(fd);
		result= false;
	}

	buf[0] = regAddr;
	memcpy(buf + 1, data, length);
	count = write(fd, buf, length + 1);
	if (count < 0) {
		_ERROR("%s Failed to write device(%d)\n", __func__, count);
		close(fd);
		result= false;
	} else if (count != length + 1) {
		_ERROR("Short write to device, expected %d, got %d\n", length + 1,
				count);
		close(fd);
		result= false;
	}
	close(fd);

	return result;
}

/**
 * write a word into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param data
 * 		a word
 *
 * @return
 *		success or failure
 *
 */
bool writeWord(unsigned char devAddr, unsigned char regAddr,
		unsigned short data) {
	return writeWords(devAddr, regAddr, 1, &data);
}

/**
 * write serveral words into the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		serveral words
 *
 * @return
 *		success or failure
 *
 */
bool writeWords(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned short* data) {

	char count = 0;
	unsigned char buf[128];
	int i; 
	int fd;
	bool result=true;

	if (length > 63) {
		_ERROR("%s: length (%d) > 63\n", __func__, length);
		result= false;
	}

	fd = open(I2C_DEV_PATH, O_RDWR);
	if (fd < 0) {
		_ERROR("%s: Failed to open device\n", __func__);
		result= false;
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		_ERROR("%s: Failed to select device\n", __func__);
		close(fd);
		result= false;
	}
	buf[0] = regAddr;
	for (i = 0; i < length; i++) {
		buf[i * 2 + 1] = data[i] >> 8;
		buf[i * 2 + 2] = data[i];
	}
	count = write(fd, buf, length * 2 + 1);
	if (count < 0) {
		_ERROR("%s: Failed to write device(%d)\n", __func__, count);
		close(fd);
		result= false;
	} else if (count != length * 2 + 1) {
		_ERROR("%s: Short write to device, expected %d, got %d\n", __func__,
				length + 1, count);
		close(fd);
		result= false;
	}
	close(fd);
	return result;
}

/**
 * read a byte from the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param data
 * 		a byte
 *
 * @return
 *		data length
 *
 */
char readByte(unsigned char devAddr, unsigned char regAddr, unsigned char *data) {
	return readBytes(devAddr, regAddr, 1, data);
}

/**
 * read serveral bytes from the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		a byte
 *
 * @return
 *		data length
 *
 */
char readBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char *data) {

	char count = 0;
	int fd = open(I2C_DEV_PATH, O_RDWR);

	if (fd < 0) {
		_ERROR("Failed to open device: \n");
		return (-1);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		_ERROR("Failed to select device: \n");
		close(fd);
		return (-1);
	}
	if (write(fd, &regAddr, 1) != 1) {
		_ERROR("Failed to write reg: \n");
		close(fd);
		return (-1);
	}
	count = read(fd, data, length);
	if (count < 0) {
		_ERROR("Failed to read device(%d): \n", count);
		close(fd);
		return (-1);
	} else if (count != length) {
		_ERROR("Short read  from device, expected %d, got %d\n", length, count);
		close(fd);
		return (-1);
	}
	close(fd);

	return count;
}

/**
 * read serveral bits from the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param bitStart
 * 		the position of the start
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		serveral bits
 *
 * @return
 *		data length
 *
 */
char readBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char *data) {

	unsigned char count, b;
	if ((count = readByte(devAddr, regAddr, &b)) != 0) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

/**
 * read a bit from the register on a i2c device
 *
 * @param devAddr
 * 		i2c address of device
 *
 * @param regAddr
 * 		address of register
 *
 * @param bitNum
 * 		the position of the bit
 *
 * @param length
 * 		length of data
 *
 * @param data
 * 		 a bit
 *
 * @return
 *		data length
 *
 */
char readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum,
		unsigned char *data) {

	unsigned char b;
	unsigned char count = readByte(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

