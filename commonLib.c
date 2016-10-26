
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "commonLib.h"

double deadband(double value, const double threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  return value;
}

