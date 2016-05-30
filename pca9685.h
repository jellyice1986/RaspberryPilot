
#include "commonLib.h"

#define PCA9685_ADDRESS 0x40			//PCA9685 i2c address

void pca9685Init();
void resetPca9685(void);
void setPWMFreq(unsigned short);
void setPWM(unsigned char, unsigned short);
bool testPca9685Connection();


