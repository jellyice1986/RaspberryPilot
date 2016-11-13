bool mpu6050Init();
unsigned char getYawPitchRollInfo(float *yprAttitude, float *yprRate,
		float *xyzAcc, float *xyzGravity, float *xyzMagnet);
float getGyroSensitivity();
float getAccSensitivity();
float getGyroSensitivityInv();
float getAccSensitivityInv();
void getMotion6RawData(short* ax, short* ay, short* az, short* gx, short* gy,
		short* gz);
void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy,
		float* gz);
void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy,
		float* gz, float* mx, float* my, float* mz);
void setYaw(float t_yaw);
void setPitch(float t_pitch);
void setRoll(float t_roll);
float getYaw();
float getPitch();
float getRoll();
void setXAcc(float x_acc);
void setYAcc(float y_acc);
void setZAcc(float z_acc);
float getXAcc();
float getYAcc();
float getZAcc();
void setYawGyro(float t_yaw_gyro);
void setPitchGyro(float t_pitch_gyro);
void setRollGyro(float t_roll_gyro);
float getYawGyro();
float getPitchGyro();
float getRollGyro();
void setXGravity(float x_gravity);
void setYGravity(float y_gravity);
void setZGravity(float z_gravity);
float getXGravity();
float getYGravity();
float getZGravity();

