
#include "commonLib.h"


extern pthread_mutex_t controlMotorMutex;

float getZAxisDegree();
void setZAxisDegree(float degree);
void setLeaveFlyControlerFlag(bool v);
bool getLeaveFlyControlerFlag();
void flyControlerInit();
void adjustMotor();
void setYawCenterPoint(float point);
float getYawCenterPoint();
float yawTransform(float originPoint);
void setGyroLimit(float v);
float getGyroLimit();
float getAngularLimit();
void setAngularLimit(float angular);
void setAdjustPeriod(unsigned short ms);
unsigned short getAdjustPeriod();
void setInitVHH(bool v);
bool getInitVHH(void);



