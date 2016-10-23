

extern pthread_mutex_t controlMotorMutex;

float getZAxisSlope();
void setZAxisSlope(float slope);
void setLeaveFlyControlerFlag(bool v);
bool getLeaveFlyControlerFlag();
void flyControlerInit();
void adjustMotor();
void setYawCenterPoint(float point);
float getYawCenterPoint();
float yawTransform(float originPoint);
void setGyroLimit(float limitation);
float getGyroLimit();
float getAngularLimit();
void setAngularLimit(float angular);
void setAdjustPeriod(unsigned short period);
unsigned short getAdjustPeriod();

#ifdef FEATURE_VH
void setInitVHH(bool v);
bool getInitVHH(void);
#endif


