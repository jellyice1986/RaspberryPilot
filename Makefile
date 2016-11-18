
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

INCLUDES = \
	-I. \
	-I${PWD}/Module/PCA9685/core/inc \
	-I${PWD}/Module/MPU6050/core/inc \
    -I${PWD}/Module/SRF02/core/inc \
	-I${PWD}/Module/MS5611/core/inc \
	-I${PWD}/Module/VL53l0x/core/inc \
	-I${PWD}/Module/VL53l0x/platform/inc

LIB_SRCS =\
	commonLib.c \
	i2c.c \
	securityMechanism.c \
	ahrs.c \
	motorControl.c \
	systemControl.c \
	pid.c \
	kalmanFilter.c \
	altHold.c \
	radioControl.c \
	flyControler.c \
	raspberryPilotMain.c

LIB_OBJS = $(LIB_SRCS:%.c=$(OBJ_DIR)/%.o)	

include $(PWD)/config.mk
RASPBERRYPILOT_CFLAGS += $(DEFAULT_CFLAGS) -Wall

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
	update-rc.d RaspberryPilot remove
	$(RM) /etc/init.d/RaspberryPilot
	cp RaspberryPilot.sh /etc/init.d/RaspberryPilot
	chmod 755 /etc/init.d/RaspberryPilot
	update-rc.d RaspberryPilot defaults

.PHONY: removeScript
removeScript:
	update-rc.d RaspberryPilot remove
	$(RM) /etc/init.d/RaspberryPilot






