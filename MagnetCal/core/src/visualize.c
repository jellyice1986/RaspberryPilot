#include "imuread.h"

MagCalibration_t magcal;

Quaternion_t current_orientation;


void apply_calibration(int16_t rawx, int16_t rawy, int16_t rawz, Point_t *out)
{
	float x, y, z;

	x = ((float)rawx * UT_PER_COUNT) - magcal.V[0];
	y = ((float)rawy * UT_PER_COUNT) - magcal.V[1];
	z = ((float)rawz * UT_PER_COUNT) - magcal.V[2];
	out->x = x * magcal.invW[0][0] + y * magcal.invW[0][1] + z * magcal.invW[0][2];
	out->y = x * magcal.invW[1][0] + y * magcal.invW[1][1] + z * magcal.invW[1][2];
	out->z = x * magcal.invW[2][0] + y * magcal.invW[2][1] + z * magcal.invW[2][2];
}

static void quad_to_rotation(const Quaternion_t *quat, float *rmatrix)
{
	float qw = quat->q0;
	float qx = quat->q1;
	float qy = quat->q2;
	float qz = quat->q3;
	rmatrix[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
	rmatrix[1] = 2.0f * qx * qy - 2.0f * qz * qw;
	rmatrix[2] = 2.0f * qx * qz + 2.0f * qy * qw;
	rmatrix[3] = 2.0f * qx * qy + 2.0f * qz * qw;
	rmatrix[4] = 1.0f  - 2.0f * qx * qx - 2.0f * qz * qz;
	rmatrix[5] = 2.0f * qy * qz - 2.0f * qx * qw;
	rmatrix[6] = 2.0f * qx * qz - 2.0f * qy * qw;
	rmatrix[7] = 2.0f * qy * qz + 2.0f * qx * qw;
	rmatrix[8] = 1.0f  - 2.0f * qx * qx - 2.0f * qy * qy;
}

static void rotate(const Point_t *in, Point_t *out, const float *rmatrix)
{
	out->x = in->x * rmatrix[0] + in->y * rmatrix[1] + in->z * rmatrix[2];
	out->y = in->x * rmatrix[3] + in->y * rmatrix[4] + in->z * rmatrix[5];
	out->z = in->x * rmatrix[6] + in->y * rmatrix[7] + in->z * rmatrix[8];
}

void display_callback(void)
{
	int i;
	//float xscale, yscale, zscale;
	//float xoff, yoff, zoff;
	float rotation[9];
	Point_t point, draw;
	Quaternion_t orientation;

	quality_reset();

#if 0
	gluLookAt(0.0, 0.0, 0.8, // eye location
		  0.0, 0.0, 0.0, // center
		  0.0, 1.0, 0.0); // up direction
#endif

#if 0
	xscale = 0.05;
	yscale = 0.05;
	zscale = 0.05;
	xoff = 0.0;
	yoff = 0.0;
	zoff = -7.0;
#endif

	//if (hard_iron.valid) {
	if (1) {
		memcpy(&orientation, &current_orientation, sizeof(orientation));
		// TODO: this almost but doesn't perfectly seems to get the
		//  real & screen axes in sync....
		//orientation.q0 *= -1.0f;
		//orientation.q1 *= -1.0f;
		//orientation.q2 *= -1.0f;
		orientation.q3 *= -1.0f;
		quad_to_rotation(&orientation, rotation);
		for (i=0; i < MAGBUFFSIZE; i++) {
			if (magcal.valid[i]) {
				apply_calibration(magcal.BpFast[0][i], magcal.BpFast[1][i],
					magcal.BpFast[2][i], &point);
				//point.x *= -1.0f;
				//point.y *= -1.0f;
				//point.z *= -1.0f;
				quality_update(&point);
				rotate(&point, &draw, rotation);
			}
		}
	}
#if 0
	printf(" quality: %5.1f  %5.1f  %5.1f  %5.1f\n",
		quality_surface_gap_error(),
		quality_magnitude_variance_error(),
		quality_wobble_error(),
		quality_spherical_fit_error());
#endif
}