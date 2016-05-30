
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "commonLib.h"


// Constrain value between min and max
float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

double deadband(double value, const double threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

