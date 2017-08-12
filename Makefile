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
RM = rm -rf 
PWD	= ${shell pwd}
SUBDIR = Module
LIB_PATH = -L$(PWD)/Module/bin
OUTPUT_DIR = bin
OBJ_DIR = obj
LIB = -lModule_RaspberryPilot -lwiringPi -lm -lpthread
PROCESS = RaspberryPilot
TARGET_PROCESS = $(OUTPUT_DIR)/$(PROCESS)
RASPBERRYPILOT_CFLAGS += $(DEFAULT_CFLAGS)

include $(PWD)/config.mk

INCLUDES = \
	-I. \
	-I${PWD}/Module/PCA9685/core/inc \
	-I${PWD}/Module/MPU6050/core/inc

LIB_SRCS = \
	commonLib.c \
	i2c.c \
	securityMechanism.c \
	ahrs.c \
	motorControl.c \
	systemControl.c \
	pid.c \
	kalmanFilter.c \
	smaFilter.c \
	altHold.c \
	radioControl.c \
	flyControler.c \
	attitudeUpdate.c\
	raspberryPilotMain.c

ifeq ($(CONFIG_ALTHOLD_MS5611_SUPPORT),y)
	INCLUDES += \
		-I${PWD}/Module/MS5611/core/inc
else	
	ifeq ($(CONFIG_ALTHOLD_SRF02_SUPPORT),y)
		INCLUDES += \
			-I${PWD}/Module/SRF02/core/inc
	else	
		ifeq ($(CONFIG_ALTHOLD_VL53L0X_SUPPORT),y)
				INCLUDES += \
					-I${PWD}/Module/VL53l0x/core/inc \
					-I${PWD}/Module/VL53l0x/platform/inc
		endif	
	endif	
endif	
	
LIB_OBJS = $(LIB_SRCS:%.c=$(OBJ_DIR)/%.o)	

.PHONY: all
all: $(TARGET_PROCESS)

$(TARGET_PROCESS): $(SUBDIR) $(LIB_OBJS)
	@echo "\033[32mMake RaspberryPilot all...\033[0m"
	mkdir -p $(dir $@)
	$(CC) $(LIB_OBJS) $(LIB) $(LIB_PATH) $(INCLUDES) -o $@

$(OBJ_DIR)/%.o: $(PWD)/%.c
	mkdir -p $(dir $@)
	@echo "\033[32mCompiling RaspberryPilot $@...\033[0m"	
	$(CC) -c -o $@ $< $(RASPBERRYPILOT_CFLAGS) $(INCLUDES) $(LIB) $(LIB_PATH) 

.PHONY: $(SUBDIR)
$(SUBDIR):
	make -C $@ 

.PHONY: clean	
clean:
	@echo "\033[32mCleaning RaspberryPilot clean...\033[0m"
	-find -name "*.o" -type f | xargs $(RM)
	-find -name "*.a" -type f | xargs $(RM)
	-find -name "$(OBJ_DIR)" -type d | xargs $(RM)
	-find -name "$(OUTPUT_DIR)" -type d | xargs $(RM)
	-find -name "$(PROCESS)" -type f | xargs $(RM)

.PHONY: updateScript
updateScript:
	@echo "\033[32mcreating RaspberryPilot.sh...\033[0m"
	sudo echo "#!/bin/bash"  >  RaspberryPilot.sh
	sudo echo "### BEGIN INIT INFO" >>  ./RaspberryPilot.sh
	sudo echo "# Provides: RaspberryPilot" >>  ./RaspberryPilot.sh
	sudo echo "# Required-Start: \$$all" >>  ./RaspberryPilot.sh
	sudo echo "# Required-Stop: \$$all" >>  ./RaspberryPilot.sh
	sudo echo "# Default-Start:  2 3 4 5" >>  ./RaspberryPilot.sh
	sudo echo "# Default-Stop: 0 1 6" >>  ./RaspberryPilot.sh
	sudo echo "# Description: Raecho spberryPilot" >>  ./RaspberryPilot.sh
	sudo echo "### END INIT INFO"  >>  ./RaspberryPilot.sh
	sudo echo " " >>  ./RaspberryPilot.sh
	sudo echo "for ((i=0;i<20;i++))do"  >>  ./RaspberryPilot.sh
	sudo echo "       result=\`ps aux  | grep 'bin/RaspberryPilot' | grep -v 'grep' | wc -l\`"  >>  ./RaspberryPilot.sh
	sudo echo "       if [ \$$result -eq 0 ];then"  >>  ./RaspberryPilot.sh
	sudo echo "                sudo $(shell pwd)/bin/RaspberryPilot &"  >>  ./RaspberryPilot.sh
	sudo echo "                echo \"retry \$$i\""  >>  ./RaspberryPilot.sh
	sudo echo "        else"  >>  ./RaspberryPilot.sh
	sudo echo "                break"  >>  ./RaspberryPilot.sh
	sudo echo "        fi"  >>  ./RaspberryPilot.sh
	sudo echo "                sleep 1;"  >>  ./RaspberryPilot.sh
	sudo echo "done"  >>  ./RaspberryPilot.sh
	@echo "\033[32updating RaspberryPilot.sh to init.d...\033[0m"
	sudo update-rc.d RaspberryPilot remove
	sudo $(RM) /etc/init.d/RaspberryPilot
	sudo cp RaspberryPilot.sh /etc/init.d/RaspberryPilot
	sudo chmod 755 /etc/init.d/RaspberryPilot
	sudo update-rc.d RaspberryPilot defaults
	sudo $(RM) ./RaspberryPilot.sh

.PHONY: removeScript
removeScript:
	@echo "\033[32removing RaspberryPilot.sh from init.d...\033[0m"				
	sudo update-rc.d RaspberryPilot remove
	sudo $(RM) /etc/init.d/RaspberryPilot






