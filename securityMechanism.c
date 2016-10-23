
#include <stdlib.h>
#include <stdio.h>
#include "motorControl.h"
#include "pid.h"
#include "securityMechanism.h"

static int packetAccCounter;

/*
*
* RaspberryPilot send packet to remote controler and add 1 to packet counter ever TRANSMIT_TIMER,  
* if RaspberryPilot receive any packet from remote controler, the counter will be set to 0, if the counter counts to MAX_COUNTER
* the security mechanism wil be triggered 
*
*/


/**
* Init security Mmechanism 
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void securityMechanismInit(){

	resetPacketCounter();	
}

/**
* decrease packet counter
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void decreasePacketCounter(){
	packetAccCounter--;
	if(packetAccCounter<MIN_COUNTER)
		packetAccCounter=0;	
}

/**
* increase packet counter
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void increasePacketCounter(){
	packetAccCounter++;
	if(packetAccCounter>MAX_COUNTER)
		packetAccCounter=MAX_COUNTER;
	//printf("getPacketCounter=%d\n",getPacketCounter());
}

/**
* reset packet counter
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void resetPacketCounter(){
	packetAccCounter=0;
}

/**
* get packet accumulation
*
* @param 
* 		void  
*
* @return 
*		packet accumulation
*
*/
int getPacketCounter(){
	return packetAccCounter<0?0:packetAccCounter;
}


/**
* trigger security mechanism
*
* @param 
* 		void  
*
* @return 
*		void
*
*/
void triggerSecurityMechanism(){

	int throttleValue=max(getMinPowerLevel(),getThrottlePowerLevel()-5);

	setPidSp(&rollAttitudePidSettings,0.f);
	setPidSp(&pitchAttitudePidSettings,0.f);
			
	setupAllMotorPoewrLevel(throttleValue,
			throttleValue, throttleValue,
			throttleValue);
	setThrottlePowerLevel(throttleValue);
}


