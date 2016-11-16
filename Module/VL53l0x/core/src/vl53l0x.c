/******************************************************************************
The vl53l0x.c in RaspberryPilot project is placed under the MIT license

Copyright (c) 2016 jellyice1986 (Tung-Cheng Wu)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "commonLib.h"
#include "vl53l0x.h"

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 1
#define VL53L0X_ADDRESS 0x29

static VL53L0X_Dev_t vl53l0xDevice;
static bool vl53l0xIsReady = false;

static VL53L0X_Error singleRangingLongRangeInit();
static void print_pal_error(VL53L0X_Error Status);

/**
 * init vl53l0x
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool vl53l0xInit() {

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Version_t Version;
	VL53L0X_Version_t *pVersion = &Version;
	VL53L0X_DeviceInfo_t DeviceInfo;
	VL53L0X_Dev_t *pVl53l0xDevice = &vl53l0xDevice;
	int32_t status_int;

	pVl53l0xDevice->I2cDevAddr = VL53L0X_ADDRESS;

	/*
	 *  Get the version of the VL53L0X API running in the firmware
	 */

	if (Status == VL53L0X_ERROR_NONE) {
		status_int = VL53L0X_GetVersion(pVersion);
		if (status_int != 0)
			Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	/*
	 *  Verify the version of the VL53L0X API running in the firmware
	 */

	if (Status == VL53L0X_ERROR_NONE) {
		if (pVersion->major != VERSION_REQUIRED_MAJOR
				|| pVersion->minor != VERSION_REQUIRED_MINOR
				|| pVersion->build != VERSION_REQUIRED_BUILD) {
			_DEBUG(DEBUG_NORMAL,
					"VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
					pVersion->major, pVersion->minor, pVersion->build,
					pVersion->revision,
					VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR,
					VERSION_REQUIRED_BUILD);
		}
	}

	if (Status == VL53L0X_ERROR_NONE) {
		_DEBUG(DEBUG_NORMAL,"Call of VL53L0X_DataInit\n");
		Status = VL53L0X_DataInit(&vl53l0xDevice); // Data initialization
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetDeviceInfo(&vl53l0xDevice, &DeviceInfo);
		if (Status == VL53L0X_ERROR_NONE) {
			_DEBUG(DEBUG_NORMAL,"VL53L0X_GetDeviceInfo:\n");
			_DEBUG(DEBUG_NORMAL,"Device Name : %s\n", DeviceInfo.Name);
			_DEBUG(DEBUG_NORMAL,"Device Type : %s\n", DeviceInfo.Type);
			_DEBUG(DEBUG_NORMAL,"Device ID : %s\n", DeviceInfo.ProductId);
			_DEBUG(DEBUG_NORMAL,"ProductRevisionMajor : %d\n",
					DeviceInfo.ProductRevisionMajor);
			_DEBUG(DEBUG_NORMAL,"ProductRevisionMinor : %d\n",
					DeviceInfo.ProductRevisionMinor);

			if ((DeviceInfo.ProductRevisionMinor != 1)
					&& (DeviceInfo.ProductRevisionMinor != 1)) {
				_DEBUG(DEBUG_NORMAL,"Error expected cut 1.1 but found cut %d.%d\n",
						DeviceInfo.ProductRevisionMajor,
						DeviceInfo.ProductRevisionMinor);
				Status = VL53L0X_ERROR_NOT_SUPPORTED;
			}
		}
		print_pal_error(Status);
	}

	//single Ranging Long Range
	if (Status == VL53L0X_ERROR_NONE)
		Status = singleRangingLongRangeInit();

	vl53l0xIsReady = ((Status == VL53L0X_ERROR_NONE) ? true : false);

	return vl53l0xIsReady;

}

/**
 * init vl53l0x single ranging and long range mode
 *
 * @param
 * 		void
 *
 * @return
 *		error message
 *
 */
VL53L0X_Error singleRangingLongRangeInit() {

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	VL53L0X_Dev_t *pDevice = &vl53l0xDevice;

	if (Status == VL53L0X_ERROR_NONE) {
		_DEBUG(DEBUG_NORMAL,"Call of VL53L0X_StaticInit\n");
		Status = VL53L0X_StaticInit(pDevice); // Device Initialization
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		_DEBUG(DEBUG_NORMAL,"Call of VL53L0X_PerformRefCalibration\n");
		Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings,
				&PhaseCal); // Device Initialization
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		_DEBUG(DEBUG_NORMAL,"Call of VL53L0X_PerformRefSpadManagement\n");
		Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount,
				&isApertureSpads); // Device Initialization
		_DEBUG(DEBUG_NORMAL,"refSpadCount = %d, isApertureSpads = %d\n", refSpadCount,
				isApertureSpads);
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {

		// no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
		_DEBUG(DEBUG_NORMAL,"Call of VL53L0X_SetDeviceMode\n");
		Status = VL53L0X_SetDeviceMode(pDevice,
				VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		print_pal_error(Status);
	}

	// Enable/Disable Sigma and Signal check
	/*
	 if (Status == VL53L0X_ERROR_NONE) {
	 Status = VL53L0X_SetSequenceStepEnable(pDevice,VL53L0X_SEQUENCESTEP_DSS, 1);
	 }
	 */

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pDevice,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				(FixPoint1616_t) (0.1 * 65536));
	}
	
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pDevice,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));
	}
	
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 33000);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pDevice,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pDevice,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}

	return Status;
}

/**
 * print error message
 *
 * @param
 * 		error type
 *
 * @return
 *		void
 *
 */
void print_pal_error(VL53L0X_Error Status) {

	char buf[VL53L0X_MAX_STRING_LENGTH];
	VL53L0X_GetPalErrorString(Status, buf);
	_DEBUG(DEBUG_NORMAL,"API Status: %i : %s\n", Status, buf);

}

/**
 * get measurement data from vl53l0x
 *
 * @param
 * 		data
 *
 * @return
 *		bool
 *
 */
bool vl53l0xGetMeasurementData(unsigned short *cm) {

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;

	VL53L0X_Dev_t *pDevice = &vl53l0xDevice;

	Status = VL53L0X_PerformSingleRangingMeasurement(pDevice,
			&RangingMeasurementData);

	*cm = RangingMeasurementData.RangeMilliMeter*0.1f;

	return ((Status == VL53L0X_ERROR_NONE) ? true : false);
}

