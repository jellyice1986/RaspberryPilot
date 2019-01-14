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
TARGET_LIB = $(OUTPUT_DIR)/libCJson_RaspberryPilot.a

include $(PWD)/../config.mk
CJSON_CFLAGS += $(DEFAULT_CFLAGS)

LIB_SRCS = \
	cJSON.c

INCLUDES = \
	-I${PWD}/..\
	-I${PWD}/core/inc
	
VPATH = \
	${PWD}/core/src
	
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
	$(CC) -g -c $(CJSON_CFLAGS) $(INCLUDES) $< -o $@

.PHONY: clean
clean:
	-${RM} -rf ./$(OUTPUT_DIR)/*  ./$(OBJ_DIR)/*


