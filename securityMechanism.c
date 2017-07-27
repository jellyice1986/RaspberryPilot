/******************************************************************************
The securityMechanism.c in RaspberryPilot project is placed under the MIT license

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
#include "commonLib.h"
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
void securityMechanismInit() {

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
void decreasePacketCounter() {
	packetAccCounter--;
	if (packetAccCounter < MIN_COUNTER)
		packetAccCounter = 0;
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
void increasePacketCounter() {
	packetAccCounter++;
	if (packetAccCounter > MAX_COUNTER)
		packetAccCounter = MAX_COUNTER;
	//_DEBUG(DEBUG_NORMAL,"getPacketCounter=%d\n",getPacketCounter());
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
void resetPacketCounter() {
	packetAccCounter = 0;
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
int getPacketCounter() {
	return packetAccCounter < 0 ? 0 : packetAccCounter;
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
void triggerSecurityMechanism() {

	setPidSp(&rollAttitudePidSettings, 0.f);
	setPidSp(&pitchAttitudePidSettings, 0.f);
	setThrottlePowerLevel(max(getMinPowerLevel(), getThrottlePowerLevel() - 10));
}

