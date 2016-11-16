/******************************************************************************
The i2c.h in RaspberryPilot project is placed under the MIT license

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

#define I2C_DEV_PATH "/dev/i2c-1"

bool checkI2cDeviceIsExist(unsigned char devAddr);
bool writeByte(unsigned char devAddr, unsigned char regAddr,
		unsigned char data);
bool writeBit(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitNum, unsigned char data);
bool writeBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char data);
bool writeBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char * data);
bool writeWord(unsigned char devAddr, unsigned char regAddr,
		unsigned short data);
bool writeWords(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned short* data);
char readByte(unsigned char devAddr, unsigned char regAddr,
		unsigned char *data);
char readBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char *data);
char readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum,
		unsigned char *data);
char readBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char *data);

