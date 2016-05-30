#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <memory.h>

#include "i2c.h"

bool checkI2cDeviceIsExist(unsigned char devAddr){
	int fd=-1;
	bool result=true;
	
	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0) {
		printf("%s: Failed to open device\n", __func__);
		result=false;
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		printf("%s: Failed to select device\n", __func__);
		result=false;
	}
	close(fd);		
	return result;
}

char writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data) {
	return writeBytes(devAddr, regAddr, 1, &data);
}

char writeBit(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitNum, unsigned char data) {
	unsigned char b;
	readByte(devAddr, regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(devAddr, regAddr, b);
}

char writeBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char data) {
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	unsigned char b;
	if (readByte(devAddr, regAddr, &b) != 0) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return writeByte(devAddr, regAddr, b);
	} else {
		return -1;
	}
}

char writeBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char * data) {

	char count = 0;
	unsigned char buf[128];
	int fd;

	if (length > 127) {
		printf("Byte write count (%d) > 127\n", length);
		return (-1);
	}

	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0) {
		printf("%s: Failed to open device\n", __func__);
		return (-1);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		printf("%s: Failed to select device\n", __func__);
		close(fd);
		return (-1);
	}
	buf[0] = regAddr;
	memcpy(buf + 1, data, length);
	count = write(fd, buf, length + 1);
	if (count < 0) {
		printf("%s Failed to write device(%d)\n", __func__, count);
		close(fd);
		return (-1);
	} else if (count != length + 1) {
		printf("Short write to device, expected %d, got %d\n", length + 1,
				count);
		close(fd);
		return (-1);
	}
	close(fd);

	return 1;
}

char writeWord(unsigned char devAddr, unsigned char regAddr,
		unsigned short data) {
	return writeWords(devAddr, regAddr, 1, &data);
}

char writeWords(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned short* data) {

	char count = 0;
	unsigned char buf[128];
	int i, fd;

	// Should do potential byteswap and call writeBytes() really, but that
	// messes with the callers buffer

	if (length > 63) {
		printf("%s: Word write count (%d) > 63\n", __func__, length);
		return (-1);
	}

	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0) {
		printf("%s: Failed to open device\n", __func__);
		return (-1);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		printf("%s: Failed to select device\n", __func__);
		close(fd);
		return (-1);
	}
	buf[0] = regAddr;
	for (i = 0; i < length; i++) {
		buf[i * 2 + 1] = data[i] >> 8;
		buf[i * 2 + 2] = data[i];
	}
	count = write(fd, buf, length * 2 + 1);
	if (count < 0) {
		printf("%s: Failed to write device(%d)\n", __func__, count);
		close(fd);
		return (-1);
	} else if (count != length * 2 + 1) {
		printf("%s: Short write to device, expected %d, got %d\n", __func__,
				length + 1, count);
		close(fd);
		return (-1);
	}
	close(fd);
	return 1;
}

char readByte(unsigned char devAddr, unsigned char regAddr, unsigned char *data) {
	return readBytes(devAddr, regAddr, 1, data);
}

char readBytes(unsigned char devAddr, unsigned char regAddr,
		unsigned char length, unsigned char *data) {

	int8_t count = 0;
	int fd = open("/dev/i2c-1", O_RDWR);

	if (fd < 0) {
		fprintf(stderr, "Failed to open device: \n");
		return (-1);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
		fprintf(stderr, "Failed to select device: \n");
		close(fd);
		return (-1);
	}
	if (write(fd, &regAddr, 1) != 1) {
		fprintf(stderr, "Failed to write reg: \n");
		close(fd);
		return (-1);
	}
	count = read(fd, data, length);
	if (count < 0) {
		fprintf(stderr, "Failed to read device(%d): \n", count);
		close(fd);
		return (-1);
	} else if (count != length) {
		fprintf(stderr, "Short read  from device, expected %d, got %d\n",
				length, count);
		close(fd);
		return (-1);
	}
	close(fd);

	return count;
}

char readBits(unsigned char devAddr, unsigned char regAddr,
		unsigned char bitStart, unsigned char length, unsigned char *data) {

	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	unsigned char count, b;
	if ((count = readByte(devAddr, regAddr, &b)) != 0) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

char readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum,
		unsigned char *data) {
	unsigned char b;
	unsigned char count = readByte(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

