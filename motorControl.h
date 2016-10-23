

/**
 *	  2 CCW2  CW2 3
 *			X
 *	  1 CW1  CCW1 0
 *			H
 */
typedef enum{
 SOFT_PWM_CCW1,
 SOFT_PWM_CW1,
 SOFT_PWM_CCW2,
 SOFT_PWM_CW2
}MOTOR_INDEX;

void motorInit();
void setupAllMotorPoewrLevel(unsigned short CW1, unsigned short CW2,
		unsigned short CCW1, unsigned short CCW2);
unsigned short getMotorPowerLevelCW1();
unsigned short getMotorPowerLevelCW2();
unsigned short getMotorPowerLevelCCW1();
unsigned short getMotorPowerLevelCCW2();
void setupCcw1MotorPoewrLevel(unsigned short CCW1);
void setupCcw2MotorPoewrLevel(unsigned short CCW2);
void setupCw1MotorPoewrLevel(unsigned short CW1);
void setupCw2MotorPoewrLevel(unsigned short CW2);
void setThrottlePowerLevel(unsigned short level);
unsigned short getThrottlePowerLevel();
unsigned short getMinPowerLevel();
unsigned short getMaxPowerLeve();
unsigned short getAdjustPowerLeveRange();
void setAdjustPowerLeveRange(int v);
unsigned short getPidAdjustPowerLimit();
void setPidAdjustPowerLimit(int v);
float getMotorGain(unsigned char index);
void setMotorGain(unsigned char index, float value);

