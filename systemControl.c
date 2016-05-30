#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include "systemControl.h"

static bool flySystemIsEnableflag;



void signalEvent(int sig);

/**
* Init paramtes and check states of raspberry pi system 
*/
bool piSystemInit() {

	if (getuid() != 0) //wiringPi requires root privileges
			{
		printf("Error: wiringPi must be run as root.\n");
		return false;
	}

	if (wiringPiSetup() == -1) {
		printf("Error:wiringPi setup failed.\n");
		return false;
	}
	disenableFlySystem();
	return true;
}


/**
*  set piSystemIsEnable to true, means remote controlle CAN start flying.
*/
void enableFlySystem() {
	flySystemIsEnableflag = true;
}

/**
*  set piSystemIsEnable to true, means remote controlle CAN NOT start flying.
*/
void disenableFlySystem() {
	flySystemIsEnableflag = false;
}


/**
*  check whether means remote controlle can start flying or not.
*/
bool flySystemIsEnable() {
	return flySystemIsEnableflag;
}


/**
*  check whether means remote controlle can start flying or not.
*/
void signalEvent(int sig) {
	setupAllMotorPoewrLevel(getMinPowerLevel(), getMinPowerLevel(),
			getMinPowerLevel(), getMinPowerLevel());
	disenableFlySystem();
	exit(EXIT_SUCCESS);
}

void setSystemSignalEvent(){
	(void)signal(SIGINT, signalEvent);
	(void) signal(SIGQUIT, signalEvent);
	(void) signal(SIGABRT, signalEvent);
}

