
#include "commonLib.h"


bool mpu6050Init();
unsigned char getYawPitchRollInfo(float *yprAttitude, float *yprRate, float *xyzAcc, float *xyzGravity,float *xyzMagnet);
void getMotion6(short* ax, short* ay, short* az, short* gx, short* gy, short* gz);
void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz,float* mx, float* my, float* mz);
void setYaw(float t_yaw);
void setPitch(float t_pitch);
void setRoll(float t_roll);
float getYaw();
float getPitch();
float getRoll();
void setYawGyro(float t_yaw_gyro);
void setPitchGyro(float t_pitch_gyro);
void setRollGyro(float t_roll_gyro);
float getYawGyro();
float getPitchGyro();
float getRollGyro();

