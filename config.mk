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

#Setup IMU is 6DOF(accelerometer+gyro) or 9DOF (accelerometer+gyro+magnetormeter)
CONFIG_MPU6050_9AXIS_SUPPORT :=y

#Choose a AHRS algorithm, only one of the following setting will be applied
#MAHONY_AHRS is appropriate for low processing MCU
#MADGWICK_AHRS is accurate at the cost of requiring extra processing powe
CONFIG_AHRS_MADGWICK_SUPPORT :=y
CONFIG_AHRS_MAHONY_AHRS_SUPPORT :=n

#Choose a sensor type for althold, only one of the following setting will be applied
CONFIG_ALTHOLD_MS5611_SUPPORT :=y
CONFIG_ALTHOLD_SRF02_SUPPORT :=n
CONFIG_ALTHOLD_VL53L0X_SUPPORT :=n

######### Don't Modify The Following Code #########

DEFAULT_CFLAGS += -O0 -Wall

ifeq ($(CONFIG_MPU6050_9AXIS_SUPPORT),y)
	DEFAULT_CFLAGS += -DMPU6050_9AXIS
else
	DEFAULT_CFLAGS += -DMPU6050_6AXIS
endif

ifeq ($(CONFIG_AHRS_MADGWICK_SUPPORT),y)
	DEFAULT_CFLAGS += -DMADGWICK_AHRS
else
	DEFAULT_CFLAGS += -DMAHONY_AHRS
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

 