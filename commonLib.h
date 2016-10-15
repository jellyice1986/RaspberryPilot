
#define true (1==1)
#define false (1==0)
#define bool char
#define LIMIT_MIN_MAX_VALUE(v,min,max) (v<=min?min:(v>=max?max:v))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

#define LOG_ENABLE 		1
#define DEBUG_GYRO 		0x0001	
#define DEBUG_ACC  		0x0002
#define DEBUG_ATTI  	0x0004
#define DEBUG_MASK 		0//(DEBUG_ACC)

#define _DEBUG(type,str,arg...) \
	if(LOG_ENABLE && (type & DEBUG_MASK)) \
		printf(str,## arg)

#define _ERROR(str,arg...) \
	if(LOG_ENABLE) \
		printf(str,## arg)

float constrain(float value, const float minVal, const float maxVal);
double deadband(double value, const double threshold);

