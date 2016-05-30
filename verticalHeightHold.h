
float getVerticalHeight();
float getVerticalSpeed();
float recordVerticalSpeed(float *xyzAcc, float *xyzGravity);
void *getDataFromMs5611Thread(void *arg);
void setVerticalHeightHoldEnable(bool v);
bool getVerticalHeightHoldEnable();
void setMaxRisenVerticalHeight(short v);
short getMaxRisenVerticalHeight();
void setStartRisenVerticalHeight(short v);
short getStartRisenVerticalHeight();


