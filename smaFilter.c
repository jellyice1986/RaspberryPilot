/******************************************************************************
The smaFilter.c in RaspberryPilot project is placed under the MIT license

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

#include "commonLib.h"
#include <string.h>
#include "smaFilter.h"

/**
 * init a  simple moving average  filter entity
 *
 * @param smaStruct
 * 		 a SMA filter entity
 *
 * @param name
 * 		 name of this entity
 *
 * @param movAgeSize
 * 		 size of window of SMA
 *
 * @return
 *		void
 *
 */
void initSmaFilterEntity(SMA_STRUCT *smaStruct,char *name, int movAgeSize){
	
	strcpy(smaStruct->name, name);
	memset(smaStruct->buf,0.f,SMA_BUFER_SIZE);
	smaStruct->movAgeSize=LIMIT_MIN_MAX_VALUE(movAgeSize,0,SMA_BUFER_SIZE);
	smaStruct->curIndex=0;
 
}

/**
 * init a  simple moving average  filter entity
 *
 * @param smaStruct
 * 		 a SMA filter entity
 *
 * @param val
 * 		 value
 *
 * @return
 *		void
 *
 */
void pushSmaData(SMA_STRUCT *smaStruct, float val) { 
	
	smaStruct->buf[smaStruct->curIndex] = val; 
	smaStruct->curIndex = (smaStruct->curIndex + 1) % smaStruct->movAgeSize;
	
}

/**
 * init a  simple moving average  filter entity
 *
 * @param smaStruct
 * 		 a SMA filter entity
 *
 * @return
 *		float
 *
 */
float pullSmaData(SMA_STRUCT *smaStruct) {  

	float sum = 0.0;  
	int i=0;
	
	for(i=0; i<smaStruct->movAgeSize; i++) {  
		sum += smaStruct->buf[i]; 
	}  
	
	return (sum /(float)smaStruct->movAgeSize);
}



