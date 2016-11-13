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
float deadband(float value, const float threshold) {

	if (fabs(value) < threshold) {
		value = 0;
	} else if (value > 0) {
		value -= threshold;
	} else if (value < 0) {
		value += threshold;
	}

	return value;
}

