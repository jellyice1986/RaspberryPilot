
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "commonLib.h"

/**
* dead band processor
*
* @param value
*               input
*
* @param threshold
*               threshold
*
* @return
*		output
*/
double deadband(double value, const double threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  return value;
}

