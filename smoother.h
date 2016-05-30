

#define SMOOTHER_BUFFER_SIZE 1

extern float roll_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
extern float pitch_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
extern float yaw_gyro_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
extern float roll_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
extern float pitch_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];
extern float yaw_attitude_smoother_buffer[SMOOTHER_BUFFER_SIZE + 1];

float getDataFromSmoother(float *buf);
void addDataToSmoother(float *buf, float value);
void initSmootherBuffer();

