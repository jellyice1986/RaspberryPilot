
PROCESS = RaspberryPilot
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
RM = rm
PWD	= ${shell pwd}
OBJ_DIR = ${PWD}
SUBDIR = Module
LIB_PATH = -L$(PWD)/Module/bin
LIB = -lModule_RaspberryPilot -lwiringPi -lm -lpthread
OBJS = \
	commonLib.o \
	i2c.o \
	securityMechanism.o \
	ahrs.o \
	motorControl.o \
	systemControl.o \
	pid.o \
	radioControl.o \
	flyControler.o \
	raspberryPilotMain.o

INCLUDES = \
	-I. \
	-I${PWD}/Module/PCA9685/core/inc \
	-I${PWD}/Module/MPU6050/core/inc \
	-I${PWD}/Module/MS5611/core/inc \
	-I${PWD}/Module/VL53l0x/core/inc \
	-I${PWD}/Module/VL53l0x/platform/inc

include $(PWD)/config.mk
RASPBERRYPILOT_CFLAGS += $(DEFAULT_CFLAGS) -Wall

.PHONY: all
all: $(SUBDIR) $(OBJS)
	@echo "\033[32mMake RaspberryPilot all...\033[0m"
	$(CC) $(OBJS) $(LIB) $(LIB_PATH) $(INCLUDES) -o $(PROCESS)

%.o: $(OBJ_DIR)/%.c
	@echo "\033[32mCompiling RaspberryPilot $@...\033[0m"	
	$(CC) -c -o $(OBJ_DIR)/$@ $< $(RASPBERRYPILOT_CFLAGS) $(INCLUDES) $(LIB) $(LIB_PATH) 

.PHONY: $(SUBDIR)
$(SUBDIR):
	make -C $@ 

.PHONY: clean	
clean:
	@echo "\033[32mCleaning RaspberryPilot clean...\033[0m"
	-find -name "*.o" | xargs rm
	-find -name "*.a" | xargs rm
	-find -name "$(PROCESS)" | xargs rm

.PHONY: updateScript
updateScript:
	update-rc.d RaspberryPilot remove
	rm -rf /etc/init.d/RaspberryPilot
	cp RaspberryPilot.sh /etc/init.d/RaspberryPilot
	chmod 755 /etc/init.d/RaspberryPilot
	update-rc.d RaspberryPilot defaults

.PHONY: removeScript
removeScript:
	update-rc.d RaspberryPilot remove
	rm -rf /etc/init.d/RaspberryPilot






