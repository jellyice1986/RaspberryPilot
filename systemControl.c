
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
#include "systemControl.h"

static bool flySystemIsEnableflag;

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

	if (getuid() != 0)
			{
		_ERROR("(%s-%d) wiringPi must be run as root\n",__func__,__LINE__);
		return false;
	}

	if (wiringPiSetup() == -1) {
		_ERROR("(%s-%d) wiringPi setup failed\n",__func__,__LINE__);
		return false;
	}
	
	(void)signal(SIGINT, signalEvent);
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
	setupAllMotorPoewrLevel(0,0,0,0);
	setThrottlePowerLevel(0);
	disenableFlySystem();
	setLeaveFlyControlerFlag(true);
	_DEBUG(DEBUG_NORMAL,"exit RaspberryPilot\n");
	usleep(2000000);
	exit(EXIT_SUCCESS);
}



