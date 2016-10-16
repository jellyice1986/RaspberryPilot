
#define true (1==1)
#define false (1==0)
#define bool char
#define LIMIT_MIN_MAX_VALUE(v,min,max) (v<=min?min:(v>=max?max:v))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

#define LOG_ENABLE 		true
#define DEBUG_NONE 		0x0
#define DEBUG_GYRO 		DEBUG_NONE//|0x0001	
#define DEBUG_ACC  		DEBUG_NONE//|0x0002
#define DEBUG_ATTI  	DEBUG_NONE//|0x0004
#define DEBUG_MASK 		(DEBUG_GYRO|DEBUG_ACC|DEBUG_ATTI)


#define _DEBUG(type,str,arg...) \
	if(LOG_ENABLE && (type & DEBUG_MASK)) \
		printf(str,## arg)

#define _ERROR(str,arg...) \
	if(LOG_ENABLE) \
		printf(str,## arg)

float constrain(float value, const float minVal, const float maxVal);
double deadband(double value, const double threshold);

