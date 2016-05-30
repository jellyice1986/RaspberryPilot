#include <stdlib.h>
#include <stdio.h>
#include "safeMachenism.h"


static int packetAccCounter;


void initPacketAccCounter(){
	resetPacketAccCounter();
	
}
void decreasePacketAccCounter(){
	packetAccCounter--;
	if(packetAccCounter<MIN_COUNTER)
		packetAccCounter=0;	
}

void increasePacketAccCounter(){
	packetAccCounter++;
	if(packetAccCounter>MAX_COUNTER)
		packetAccCounter=MAX_COUNTER;
	//printf("getPacketAccCounter=%d\n",getPacketAccCounter());
}

void resetPacketAccCounter(){
	packetAccCounter=0;
}

int getPacketAccCounter(){
	return packetAccCounter<0?0:packetAccCounter;
}
