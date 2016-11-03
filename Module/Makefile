
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
PWD	= ${shell pwd}
RM = rm
OUTPUT_DIR = bin
OBJ_DIR = obj
TARGET_LIB = $(OUTPUT_DIR)/libModule_RaspberryPilot.a

include $(PWD)/../config.mk
MODULE_CFLAGS += $(DEFAULT_CFLAGS) -O0 -Wall

LIB_SRCS = \
	pca9685.c \
	mpu6050.c \
	ms5611.c\
	vl53l0x.c \
	vl53l0x_api_calibration.c \
	vl53l0x_api_core.c \
	vl53l0x_api_ranging.c \
	vl53l0x_api_strings.c \
	vl53l0x_api.c \
	vl53l0x_platform.c
	
INCLUDES = \
	-I${PWD}/..\
	-I${PWD}/PCA9685/core/inc \
	-I${PWD}/MPU6050/core/inc \
	-I${PWD}/MS5611/core/inc \
	-I${PWD}/VL53l0x/core/inc \
	-I${PWD}/VL53l0x/platform/inc

VPATH = \
	${PWD}/PCA9685/core/src \
	${PWD}/MPU6050/core/src \
	${PWD}/MS5611/core/src \
	${PWD}/VL53l0x/core/src \
	${PWD}/VL53l0x/platform/src/

LIB_OBJS  = $(LIB_SRCS:%.c=$(OBJ_DIR)/%.o)

.PHONY: all
all: ${TARGET_LIB}

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


