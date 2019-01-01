/******************************************************************************
 The systemControl.c in RaspberryPilot project is placed under the MIT license

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
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <wiringPi.h>
#include "commonLib.h"
#include "motorControl.h"
#include "flyControler.h"
#include "radioControl.h"
#include "systemControl.h"

static bool flySystemIsEnableflag;
static bool magnetCalibrationIsEnableflag;

void signalEvent(int sig);

/**
 * Init Raspberry Pi
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
bool piSystemInit() {

	if (getuid() != 0) {
		_ERROR("(%s-%d) wiringPi must be run as root\n", __func__, __LINE__);
		return false;
	}

	if (wiringPiSetup() == -1) {
		_ERROR("(%s-%d) wiringPi setup failed\n", __func__, __LINE__);
		return false;
	}

	(void) signal(SIGINT, signalEvent);
	(void) signal(SIGQUIT, signalEvent);
	(void) signal(SIGABRT, signalEvent);
	(void) signal(SIGALRM, signalEvent);

	return true;
}

/**
 * set piSystemIsEnable become true to indicate that RaspberryPilot CAN start flying
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
void enableFlySystem() {
	flySystemIsEnableflag = true;
}

/**
 * set piSystemIsEnable become false to indicate that RaspberryPilot CAN NOT start flying
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
void disenableFlySystem() {
	flySystemIsEnableflag = false;
}

/**
 *
 *  check whether RaspberryPilot can start flying or not
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
bool flySystemIsEnable() {
	return flySystemIsEnableflag;
}

/**
 *
 *  check whether magnet calibration mode is enabled or not
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
bool magnetCalibrationIsEnable(){
	return magnetCalibrationIsEnableflag;
}

/**
 * set flag to indicate magnet calibration mode is enabled
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
void enableMagnetCalibration() {
	magnetCalibrationIsEnableflag = true;
}

/**
 * set flag to indicate magnet calibration mode is disable
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
void disenableMagnetCalibration() {
	magnetCalibrationIsEnableflag = false;
}

/**
 *
 * signal event handeler
 *
 * @param
 *		void
 *
 * @return
 *		bool
 *
 */
void signalEvent(int sig) {

	setThrottlePowerLevel(0);
	setupAllMotorPoewrLevel(0, 0, 0, 0);
	disenableFlySystem();
	setLeaveFlyControlerFlag(true);
	_DEBUG(DEBUG_NORMAL, "Exit RaspberryPilot\n");
	closeRadio();
	usleep(1000000);
	exit(EXIT_SUCCESS);
}

