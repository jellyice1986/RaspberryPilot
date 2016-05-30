
#define true (1==1)
#define false (1==0)
#define bool char
#define LIMIT_MIN_MAX_VALUE(v,min,max) (v<=min?min:(v>=max?max:v))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

float constrain(float value, const float minVal, const float maxVal);
double deadband(double value, const double threshold);

