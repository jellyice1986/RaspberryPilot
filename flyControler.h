/******************************************************************************
 The flyControler.h in RaspberryPilot project is placed under the MIT license

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

#define FLIP_NONE 		0x0
#define FLIP_RIGHT 		0x1
#define FLIP_LEFT 		0x2
#define FLIP_FRONT 		0x4
#define FLIP_BACK 		0x8

extern pthread_mutex_t controlMotorMutex;

void setLeaveFlyControlerFlag(bool v);
bool getLeaveFlyControlerFlag();
bool flyControlerInit();
void motorControler();
void motorControlerFlipping();
void setYawCenterPoint(float point);
float getYawCenterPoint();
float yawTransform(float originPoint);
void setGyroLimit(float limitation);
float getGyroLimit();
float getAngularLimit();
void setAngularLimit(float angular);
void setAdjustPeriod(unsigned short period);
unsigned short getAdjustPeriod();
void setAltitudePidOutputLimitation(float v);
float getAltitudePidOutputLimitation(void);
unsigned char getFlippingFlag();
void setFlippingFlag(unsigned char val);
unsigned char getFlippingStep();
void setFlippingStep(unsigned char val);
bool getFlippingIsEnable();
void setFlippingIsEnable(bool val);
unsigned short getFlipPower();
unsigned char getFlipThreadHold();

