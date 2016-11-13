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

