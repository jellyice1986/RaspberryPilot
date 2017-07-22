# /******************************************************************************
# The Makefile in RaspberryPilot project is placed under the MIT license
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

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
PWD	= ${shell pwd}
RM = rm
OUTPUT_DIR = bin
OBJ_DIR = obj
TARGET_LIB = $(OUTPUT_DIR)/libModule_RaspberryPilot.a

include $(PWD)/../config.mk
MODULE_CFLAGS += $(DEFAULT_CFLAGS)

LIB_SRCS = \
	pca9685.c \
	mpu6050.c 

INCLUDES = \
	-I${PWD}/..\
	-I${PWD}/PCA9685/core/inc \
	-I${PWD}/MPU6050/core/inc
	
VPATH = \
	${PWD}/PCA9685/core/src \
	${PWD}/MPU6050/core/src
	
ifeq ($(CONFIG_ALTHOLD_MS5611_SUPPORT),y)
	LIB_SRCS += \
		ms5611.c
		
	INCLUDES += \
		-I${PWD}/MS5611/core/inc 
		
	VPATH += \
		${PWD}/MS5611/core/src
else
	ifeq ($(CONFIG_ALTHOLD_SRF02_SUPPORT),y)
		LIB_SRCS += \
			srf02.c
			
		INCLUDES += \
			 -I${PWD}/SRF02/core/inc
		
		VPATH += \
			${PWD}/SRF02/core/src
	else	
		ifeq ($(CONFIG_ALTHOLD_VL53L0X_SUPPORT),y)
			LIB_SRCS += \
				vl53l0x.c \
				vl53l0x_api_calibration.c \
				vl53l0x_api_core.c \
				vl53l0x_api_ranging.c \
				vl53l0x_api_strings.c \
				vl53l0x_api.c \
				vl53l0x_platform.c
			
			INCLUDES += \
				-I${PWD}/VL53l0x/core/inc \
				-I${PWD}/VL53l0x/platform/inc
			
			VPATH += \
				${PWD}/VL53l0x/core/src \
				${PWD}/VL53l0x/platform/src
		endif	
	endif	
endif

LIB_OBJS  = $(LIB_SRCS:%.c=$(OBJ_DIR)/%.o)

.PHONY: all
all: $(TARGET_LIB)

$(TARGET_LIB): $(LIB_OBJS)
	@echo "\033[32mCompiling RaspberryPilot $@...\033[0m"
	mkdir -p $(dir $@)
	$(AR) -rcs $@ $^

$(OBJ_DIR)/%.o:%.c
	@echo "\033[32mCompiling RaspberryPilot $@...\033[0m"
	mkdir -p $(dir $@)
	$(CC) -g -c $(MODULE_CFLAGS) $(INCLUDES) $< -o $@

.PHONY: clean
clean:
	-${RM} -rf ./$(OUTPUT_DIR)/*  ./$(OBJ_DIR)/*


