
#include "commonLib.h"

bool checkI2cDeviceIsExist(unsigned char devAddr);

char writeByte(unsigned char devAddr, unsigned char regAddr,
		unsigned char data);
char writeBit(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitNum, unsigned char data);
char writeBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char data);
char writeBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char * data);
char writeWord(unsigned char devAddr, unsigned char regAddr,
		unsigned short data);
char writeWords(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned short* data);
char readByte(unsigned char devAddr, unsigned char regAddr,
		unsigned char *data);
char readBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char *data);
char readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum,
		unsigned char *data);
char readBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char *data);

