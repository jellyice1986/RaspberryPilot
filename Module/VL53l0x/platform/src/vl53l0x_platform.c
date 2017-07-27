/******************************************************************************
 The vl53l0x_platform.c in RaspberryPilot project is placed under the MIT license

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
#include "commonLib.h"
#include "i2c.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	return Status;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	return Status;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
		uint32_t count) {

	bool ret = writeBytes(Dev->I2cDevAddr, index, count, pdata);

	if (!ret) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
		uint32_t count) {

	int ret = readBytes(Dev->I2cDevAddr, index, count, pdata);

	if (ret < 0) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {

	bool ret = writeByte(Dev->I2cDevAddr, index, data);

	if (!ret) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {

	bool ret = writeWord(Dev->I2cDevAddr, index, data);

	if (!ret) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;

}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {

	uint16_t pData[2];
	bool ret;

	pData[0] = (uint16_t) (data & 0x0000FFFF);
	pData[1] = (uint16_t) ((data & 0xFFFF0000) >> 16);

	ret = writeWords(Dev->I2cDevAddr, index, 2, pData);

	if (!ret) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;

}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index,
		uint8_t AndData, uint8_t OrData) {

	uint8_t data;
	int ret = readByte(Dev->I2cDevAddr, index, &data);

	if (ret < 0) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	data = (data & AndData) | OrData;

	ret = writeByte(Dev->I2cDevAddr, index, data);

	if (!ret) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;

}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {

	int ret = readByte(Dev->I2cDevAddr, index, data);

	if (ret < 0) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {

	uint8_t buf[2];
	int ret = readBytes(Dev->I2cDevAddr, index, 2, buf);
	uint16_t tmp = 0;

	tmp |= buf[1] << 0;
	tmp |= buf[0] << 8;

	*data = tmp;

	if (ret < 0) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
	uint8_t buf[4];
	int ret = readBytes(Dev->I2cDevAddr, index, 4, buf);
	uint32_t tmp = 0;

	tmp |= buf[3] << 0;
	tmp |= buf[2] << 8;
	tmp |= buf[1] << 16;
	tmp |= buf[0] << 24;
	*data = tmp;

	if (ret < 0) {
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {

	usleep(5000);

	return VL53L0X_ERROR_NONE;
}
