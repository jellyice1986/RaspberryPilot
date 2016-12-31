/******************************************************************************
The commonLib.h in RaspberryPilot project is placed under the MIT license

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

#define true (1==1)
#define false (1==0)
#define bool char
#define DE_TO_RA 0.01745329251f  // PI/180
#define RA_TO_DE 57.29577951307f // 182/PI

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#define LIMIT_MIN_MAX_VALUE(value,minVal,maxVal) (min(maxVal, max(minVal,value)))
#define NON_ZERO(value) (value==0?1:value)


#define LOG_ENABLE 						true
#define DEBUG_NONE 						0x0

#define DEBUG_ENABLE					true
#define DEBUG_NORMAL                    DEBUG_NONE|0x00000001
#define DEBUG_GYRO 						DEBUG_NONE//|0x00000002	
#define DEBUG_ACC  						DEBUG_NONE//|0x00000004
#define DEBUG_ATTITUDE					DEBUG_NONE//|0x00000008
#define DEBUG_IMUUPDATE_INTVAL  		DEBUG_NONE//|0x00000010
#define DEBUG_ATTITUDE_PID_OUTPUT  		DEBUG_NONE//|0x00000020
#define DEBUG_RATE_PID_OUTPUT  			DEBUG_NONE//|0x00000040
#define DEBUG_MASK 						(DEBUG_NORMAL|DEBUG_GYRO|DEBUG_ACC|DEBUG_ATTITUDE|\
										DEBUG_IMUUPDATE_INTVAL|DEBUG_ATTITUDE_PID_OUTPUT|DEBUG_RATE_PID_OUTPUT)
#define _DEBUG(type,str,arg...) do{ if(LOG_ENABLE && DEBUG_ENABLE && ((type) & DEBUG_MASK)) printf(str,## arg);}while(0)

#define DEBUG_HOVER_ENABLE 	  		   true
#define DEBUG_HOVER_NORMAL             DEBUG_NONE|0x00000001
#define DEBUG_HOVER_ALT_SPEED          DEBUG_NONE//|0x00000002
#define DEBUG_HOVER_ACC_SPEED          DEBUG_NONE//|0x00000004
#define DEBUG_HOVER_SPEED          	   DEBUG_NONE//|0x00000008
#define DEBUG_HOVER_RAW_ALTITUDE  	   DEBUG_NONE//|0x00000010
#define DEBUG_HOVER_MASK 			   (DEBUG_HOVER_NORMAL|DEBUG_HOVER_ALT_SPEED|DEBUG_HOVER_ACC_SPEED|DEBUG_HOVER_SPEED|\
										DEBUG_HOVER_RAW_ALTITUDE)
#define _DEBUG_HOVER(type,str,arg...) do{ if(LOG_ENABLE && DEBUG_HOVER_ENABLE && ((type) & DEBUG_HOVER_MASK)) printf(str,## arg);}while(0)

#define _ERROR(str,arg...) do{ if(LOG_ENABLE) printf(str,## arg);}while(0)

float deadband(float value, const float threshold);

