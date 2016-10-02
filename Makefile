

PROCESS = RaspberryPilot
CC = gcc
RM = rm -f
PWD	= ${shell pwd}
OBJ_DIR = ${PWD}
CFLAGS = -DMPU6050_9AXIS -DMPU_DMP_YAW
OBJS =    commonLib.o i2c.o pca9685.o safeMachenism.o ahrs.o motorControl.o mpu6050.o ms5611.o verticalHeightHold.o systemControl.o pid.o radioControl.o flyControler.o raspberryPilotMain.o
HEADERS = commonLib.h i2c.h pca9685.h safeMachenism.h ahrs.h motorControl.h mpu6050.h ms5611.h verticalHeightHold.h systemControl.h pid.h radioControl.h flyControler.h

all: $(OBJS) $(HEADERS)
	@echo -e "\033[32m Make RaspberryPilot all...\033[0m"
	sudo ${CC} -o ${PROCESS} ${OBJS} -lwiringPi -lm -lpthread ${CFLAGS}

%.o: $(OBJ_DIR)/%.c
	@echo -e "\033[32mCompiling RaspberryPilot $@...\033[0m"	
	${CC} -c -o ${OBJ_DIR}/$@ $< ${CFLAGS}

clean:
	@echo -e "\033[32mCleaning RaspberryPilot clean...\033[0m"
	sudo ${RM} *.o  ${PROCESS}

updateScript:
	sudo update-rc.d RaspberryPilot remove
	sudo rm -rf /etc/init.d/RaspberryPilot
	sudo cp RaspberryPilot.sh /etc/init.d/RaspberryPilot
	sudo chmod 755 /etc/init.d/RaspberryPilot
	sudo update-rc.d RaspberryPilot defaults

removeScript:
	sudo update-rc.d RaspberryPilot remove
	sudo rm -rf /etc/init.d/RaspberryPilot






