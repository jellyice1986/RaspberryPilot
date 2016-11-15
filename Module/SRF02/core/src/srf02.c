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


