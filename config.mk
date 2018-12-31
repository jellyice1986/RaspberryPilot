# /******************************************************************************
# The config.mk in RaspberryPilot project is placed under the MIT license
#
# Copyright (c) 2016 jellyice1986 (Tung-Cheng Wu)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ******************************************************************************/

#Choose IMU is 6DOF(accelerometer+gyro) or 9DOF (accelerometer+gyro+magnetormeter)
#set up this flag to n if you haven't calibrated your magnetometer
CONFIG_MPU6050_9AXIS_SUPPORT :=y

#Define the PCA9685 channel which is used to generate PWM signal to the ESCs at CCW1,CCW2,CW1 and CW2
#
# 	  (motor#2) CCW2    CW2  (motor#3)
# 			 X
# 	  (motor#1)  CW1    CCW1 (motor#0)
#	    		Front
CONFIG_ESC_PCA9685_CHANNEL_CCW1 :=0
CONFIG_ESC_PCA9685_CHANNEL_CW1  :=1
CONFIG_ESC_PCA9685_CHANNEL_CCW2 :=2
CONFIG_ESC_PCA9685_CHANNEL_CW2  :=3

#Choose a AHRS algorithm, only one of the following setting will be applied
#MAHONY_AHRS is appropriate for low processing MCU
#MADGWICK_AHRS is accurate at the cost of requiring extra processing powe
CONFIG_AHRS_MADGWICK_SUPPORT :=y
CONFIG_AHRS_MAHONY_SUPPORT   :=n

#Set up ESC protocol and update rate
#RaspberryPilot supports the following protocols:
#Standard PWM: 50-490 Hz update rate, 1000-2000 us pulse width
#PWMSync: 500 Hz update rate, 1000-1900 us pulse width
#OneShot125: 500-2000 HZ update rate, 125-250 us pulse width 
#set up ESC update rate
CONFIG_ESC_UPDATE_RATE_SUPPORT  :=490
#Only one of the following setting will be applied
CONFIG_ESC_STANDARD_PWM_SUPPORT :=y
CONFIG_ESC_PWM_SYNC_SUPPORT 	:=n
CONFIG_ESC_ONESHOT125_SUPPORT   :=n

#Choose a sensor type for althold, only one of the following setting will be applied
CONFIG_ALTHOLD_MS5611_SUPPORT  :=y
CONFIG_ALTHOLD_SRF02_SUPPORT   :=n
CONFIG_ALTHOLD_VL53L0X_SUPPORT :=n

######### Don't Modify The Following Code #########

DEFAULT_CFLAGS += -O0 -Wall

ifeq ($(CONFIG_MPU6050_9AXIS_SUPPORT),y)
	DEFAULT_CFLAGS += -DMPU6050_9AXIS
else
	DEFAULT_CFLAGS += -DMPU6050_6AXIS
endif

DEFAULT_CFLAGS += -DSOFT_PWM_CCW1=$(CONFIG_ESC_PCA9685_CHANNEL_CCW1)
DEFAULT_CFLAGS += -DSOFT_PWM_CW1=$(CONFIG_ESC_PCA9685_CHANNEL_CW1)
DEFAULT_CFLAGS += -DSOFT_PWM_CCW2=$(CONFIG_ESC_PCA9685_CHANNEL_CCW2)
DEFAULT_CFLAGS += -DSOFT_PWM_CW2=$(CONFIG_ESC_PCA9685_CHANNEL_CW2)

ifeq ($(CONFIG_AHRS_MADGWICK_SUPPORT),y)
	DEFAULT_CFLAGS += -DMADGWICK_AHRS
else
	DEFAULT_CFLAGS += -DMAHONY_AHRS
endif

ifeq ($(CONFIG_ESC_ONESHOT125_SUPPORT),y)
	DEFAULT_CFLAGS += -DESC_ONESHOT125
	DEFAULT_CFLAGS += -DESC_UPDATE_RATE=$(CONFIG_ESC_UPDATE_RATE_SUPPORT)
else
	ifeq ($(CONFIG_ESC_PWM_SYNC_SUPPORT),y)
		DEFAULT_CFLAGS += -DESC_PWM_SYNC
		DEFAULT_CFLAGS += -DESC_UPDATE_RATE=500
	else
		DEFAULT_CFLAGS += -DESC_STANDARD_PWM
		DEFAULT_CFLAGS += -DESC_UPDATE_RATE=$(CONFIG_ESC_UPDATE_RATE_SUPPORT)
	endif
endif

ifeq ($(CONFIG_ALTHOLD_MS5611_SUPPORT),y)
	DEFAULT_CFLAGS += -DALTHOLD_MODULE_MS5611
else	
	ifeq ($(CONFIG_ALTHOLD_SRF02_SUPPORT),y)
		DEFAULT_CFLAGS += -DALTHOLD_MODULE_SRF02
	else	
		ifeq ($(CONFIG_ALTHOLD_VL53L0X_SUPPORT),y)
			DEFAULT_CFLAGS += -DALTHOLD_MODULE_VL53L0X
		endif	
	endif	
endif

 