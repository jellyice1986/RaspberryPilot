/******************************************************************************
 The mpu6050.c in RaspberryPilot project is placed under the MIT license

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

 =============================================================

 ATTENTION:

 The code in this file is mostly copied and rewritten from

 Jeff Rowberg:
 https://github.com/jrowberg/i2cdevlib

 ******************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <memory.h>
#include "commonLib.h"
#include "i2c.h"
#include "kalmanFilter.h"
#include "mpu6050.h"

#define MPU9150_MPU9250_RA_MAG_ADDRESS		0x0C
#define MPU9150_MPU9250_RA_MAG_XOUT_L		0x03
#define MPU9150_MPU9250_RA_MAG_XOUT_H		0x04
#define MPU9150_MPU9250_RA_MAG_YOUT_L		0x05
#define MPU9150_MPU9250_RA_MAG_YOUT_H		0x06
#define MPU9150_MPU9250_RA_MAG_ZOUT_L		0x07
#define MPU9150_MPU9250_RA_MAG_ZOUT_H		0x08
#define MPU9150_MPU9250_RA_INT_PIN_CFG      0x37
#define AUTO_CAL_BUFFER_SIZE 10
#define AUTO_CAL_GYRO_DEADZONE 0.1
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_ADDRESS     		MPU6050_ADDRESS_AD0_LOW 
#define MPU9150_RA_MAG_ADDRESS		0x0C
#define MPU9150_RA_MAG_XOUT_L		0x03
#define MPU9150_RA_MAG_XOUT_H		0x04
#define MPU9150_RA_MAG_YOUT_L		0x05
#define MPU9150_RA_MAG_YOUT_H		0x06
#define MPU9150_RA_MAG_ZOUT_L		0x07
#define MPU9150_RA_MAG_ZOUT_H		0x08
#define MPU9150_RA_INT_PIN_CFG      0x37
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW  //MPU6050_ADDRESS_AD0_LOW
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0
#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1
#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3
#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06
#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03
#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03
#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07
#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0
#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4
#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF
#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4
#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5
#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0
#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0
#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01
#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01
#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01
#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01
#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0
#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0
#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0
#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0
#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2
#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0
#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0
#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3
#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5
#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

static unsigned char devAddr;
static unsigned char scaleGyroRange;
static unsigned char scaleAccRange;
static unsigned char buffer[14];
static short xGyroOffset;
static short yGyroOffset;
static short zGyroOffset;

void getMotion6RawData(short* ax, short* ay, short* az, short* gx, short* gy,
		short* gz);
void setClockSource(unsigned char source);
void setFullScaleGyroRange(unsigned char range);
void setFullScaleAccelRange(unsigned char range);
void setSleepEnabled(unsigned char enabled);
void setMemoryStartAddress(unsigned char address);
char getXGyroOffset();
void setXGyroOffset(char offset);
char getYGyroOffset();
void setYGyroOffset(char offset);
char getZGyroOffset();
void setZGyroOffset(char offset);
void setXGyroOffsetUser(short offset);
void setYGyroOffsetUser(short offset);
void setZGyroOffsetUser(short offset);
void setSlaveAddress(unsigned char num, unsigned char address);
void setI2CMasterModeEnabled(unsigned char enabled);
void setI2CBypassEnabled(char enabled);
void setIntEnabled(unsigned char enabled);
void setRate(unsigned char rate);
void setDLPFMode(unsigned char mode);
void setMotionDetectionThreshold(unsigned char threshold);
void setZeroMotionDetectionThreshold(unsigned char threshold);
unsigned char getMotionDetectionDuration();
void setMotionDetectionDuration(unsigned char duration);
unsigned char getZeroMotionDetectionDuration();
void setZeroMotionDetectionDuration(unsigned char duration);
unsigned char getIntStatus();
void setExternalFrameSync(unsigned char sync);
void reset();
void setXGyroOffsetTC(char offset);
void setYGyroOffsetTC(char offset);
void setZGyroOffsetTC(char offset);

/**
 * Init MPU6050
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
bool mpu6050Init() {

	if (checkI2cDeviceIsExist(MPU6050_ADDRESS_AD0_LOW)) {
		devAddr = MPU6050_ADDRESS_AD0_LOW;
		_DEBUG(DEBUG_NORMAL, "MPU6050 exist\n");
	} else if (checkI2cDeviceIsExist(MPU6050_ADDRESS_AD0_HIGH)) {
		devAddr = MPU6050_ADDRESS_AD0_HIGH;
		_DEBUG(DEBUG_NORMAL, "MPU6050 exist\n");
	} else {
		_DEBUG(DEBUG_NORMAL, "MPU6050 dowsn't exist\n");
		return false;
	}

	scaleGyroRange = 0;
	scaleAccRange = 0;
	xGyroOffset = 22;  //pitch
	yGyroOffset = -15;  // row
	zGyroOffset = 4; //yaw

	_DEBUG(DEBUG_NORMAL, "Resetting MPU6050 ...\n");
	reset();
	usleep(120000); // wait after reset

	_DEBUG(DEBUG_NORMAL, "Disabling sleep mode...\n");
	setSleepEnabled(false);
	usleep(1000);

	_DEBUG(DEBUG_NORMAL, "Reading gyro offset values...\n");
	char xgOffset = getXGyroOffset();
	char ygOffset = getYGyroOffset();
	char zgOffset = getZGyroOffset();
	_DEBUG(DEBUG_NORMAL, "X gyro offset = %d\n", xgOffset);
	_DEBUG(DEBUG_NORMAL, "Y gyro offset = %d\n", ygOffset);
	_DEBUG(DEBUG_NORMAL, "Z gyro offset = %d\n", ygOffset);

	_DEBUG(DEBUG_NORMAL, "Setting sample rate to 1k Hz...\n");
	setRate(0); // 1khz / (1 + 0) = 1k Hz

	_DEBUG(DEBUG_NORMAL, "Setting clock source to Z Gyro...\n");
	setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	usleep(1000);

	_DEBUG(DEBUG_NORMAL, "Setting DLPF bandwidth to 184Hz...\n");
	setDLPFMode(MPU6050_DLPF_BW_188);
	usleep(1000);

	_DEBUG(DEBUG_NORMAL, "Setting gyro sensitivity to +/- 2000 deg/sec...\n");
	setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	usleep(1000);

	_DEBUG(DEBUG_NORMAL, "Setting X/Y/Z gyro offsets to previous values...\n");
	setXGyroOffsetTC(xgOffset);
	setYGyroOffsetTC(ygOffset);
	setZGyroOffsetTC(zgOffset);

	_DEBUG(DEBUG_NORMAL, "Setting accel sensitivity to +/- 8 g...\n");
	setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
	usleep(1000);

	_DEBUG(DEBUG_NORMAL, "Setting X/Y/Z gyro user offsets to %d/%d/%d...\n",
			xGyroOffset, yGyroOffset, zGyroOffset);
	setXGyroOffsetUser(xGyroOffset);
	usleep(1000);
	setYGyroOffsetUser(yGyroOffset);
	usleep(1000);
	setZGyroOffsetUser(zGyroOffset);
	usleep(1000);

#ifdef MPU6050_9AXIS
	_DEBUG(DEBUG_NORMAL,"setup AK8963\n");
	_DEBUG(DEBUG_NORMAL,"Disable MPU6050 master mode\n");
	setI2CMasterModeEnabled(false);
	usleep(10000);

	_DEBUG(DEBUG_NORMAL,"Enable MPU6050 bypass mode\n");
	setI2CBypassEnabled(true);
	usleep(10000);

	if (checkI2cDeviceIsExist(MPU9150_RA_MAG_ADDRESS)) {
		_DEBUG(DEBUG_NORMAL, "AK8963 exist\n");
	} else {
		_DEBUG(DEBUG_NORMAL, "AK8963 dowsn't exist\n");
		return false;
	}

	_DEBUG(DEBUG_NORMAL,"Reset AK8963\n");
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0B, 0x01); // Reset Device
	usleep(10000);
	_DEBUG(DEBUG_NORMAL,"Setup power down mode and full scale mode (16 bits) \n");
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x00|0x10);// power down mode|Full Scale
	usleep(10000);

#endif

	return true;

}

/**
 * get sensitivity of gyro
 *
 * @param
 * 		void
 *
 * @return
 *		sensitivity
 *
 */
float getGyroSensitivity() {
	switch (scaleGyroRange) {
	case MPU6050_GYRO_FS_250:
		return 131.f;
		break;
	case MPU6050_GYRO_FS_500:
		return 65.5f;
		break;
	case MPU6050_GYRO_FS_1000:
		return 32.8f;
		break;
	case MPU6050_GYRO_FS_2000:
		return 16.4f;
		break;
	default:
		_ERROR("(%s-%d) Unknow Gyro index\n", __func__, __LINE__);
		return -1.f;
	}
}

/**
 * get sensitivity of acceleration
 *
 * @param
 * 		void
 *
 * @return
 *		sensitivity
 *
 */
float getAccSensitivity() {
	switch (scaleAccRange) {
	case MPU6050_ACCEL_FS_2:
		return 16384.f;
		break;
	case MPU6050_ACCEL_FS_4:
		return 8192.f;
		break;
	case MPU6050_ACCEL_FS_8:
		return 4096.f;
		break;
	case MPU6050_ACCEL_FS_16:
		return 2048.f;
		break;
	default:
		_ERROR("(%s-%d) Unknow acc index\n", __func__, __LINE__);
		return -1.f;
	}
}

/**
 * get the reciprocal of a sensitivity of gyro
 *
 * @param
 * 		void
 *
 * @return
 *		sensitivity
 *
 */
float getGyroSensitivityInv() {
	switch (scaleGyroRange) {
	case MPU6050_GYRO_FS_250:
		return 0.0076335877862f;
		break;
	case MPU6050_GYRO_FS_500:
		return 0.0152671755725f;
		break;
	case MPU6050_GYRO_FS_1000:
		return 0.0304878048780f;
		break;
	case MPU6050_GYRO_FS_2000:
		return 0.0609756097560f;
		break;
	default:
		_ERROR("(%s-%d) Unknow Gyro index\n", __func__, __LINE__);
		return -1.f;
	}
}

/**
 * get the reciprocal of a sensitivity of acceleration
 *
 * @param
 * 		void
 *
 * @return
 *		sensitivity
 *
 */
float getAccSensitivityInv() {

	switch (scaleAccRange) {
	case MPU6050_ACCEL_FS_2:
		return 0.0000610351563f;
		break;
	case MPU6050_ACCEL_FS_4:
		return 0.0001220703125f;
		break;
	case MPU6050_ACCEL_FS_8:
		return 0.000244140625f;
		break;
	case MPU6050_ACCEL_FS_16:
		return 0.00048828125f;
		break;
	default:
		_ERROR("(%s-%d) Unknow acc index\n", __func__, __LINE__);
		return -1.f;
	}
}

/** 
 * Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source
 *		 New clock source setting
 *
 * @return
 *		void
 *
 */
void setClockSource(unsigned char source) {
	writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
	MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** 
 * Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @param 
 *		void
 *
 * @return 
 * 		Current full-scale gyroscope range setting
 *
 */
unsigned char getFullScaleGyroRange() {
	readBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
	MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
	return buffer[0];
}

/** 
 * Set full-scale gyroscope range.
 *
 * @param range
 *		 New full-scale gyroscope range value
 *
 * @return 
 * 		void
 *
 */
void setFullScaleGyroRange(unsigned char range) {
	writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
	MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	scaleGyroRange = range;
}

/**
 * Set full-scale accelerometer range.
 *
 * @param range
 *		 New full-scale accelerometer range setting
 *
 * @return 
 * 		void
 *
 */
void setFullScaleAccelRange(unsigned char range) {
	writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
	MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	scaleAccRange = range;
}

/** 
 * Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 *
 * @param ax
 *		 16-bit signed integer container for accelerometer X-axis value
 *
 * @param ay
 *		 16-bit signed integer container for accelerometer Y-axis value
 *
 * @param az
 *		 16-bit signed integer container for accelerometer Z-axis value
 *
 * @param gx
 *		 16-bit signed integer container for gyroscope X-axis value
 *
 * @param gy
 *		 16-bit signed integer container for gyroscope Y-axis value
 *
 * @param gz
 *		 16-bit signed integer container for gyroscope Z-axis value
 *
 * @return 
 * 		void
 *
 */
void getMotion6RawData(short* ax, short* ay, short* az, short* gx, short* gy,
		short* gz) {
	readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
	*ax = (((short) buffer[0]) << 8) | buffer[1];
	*ay = (((short) buffer[2]) << 8) | buffer[3];
	*az = (((short) buffer[4]) << 8) | buffer[5];
	readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
	*gx = (((short) buffer[0]) << 8) | buffer[1];
	*gy = (((short) buffer[2]) << 8) | buffer[3];
	*gz = (((short) buffer[4]) << 8) | buffer[5];
}

/** 
 * Set sleep mode status.
 * 
 * @param enabled
 *		 New sleep mode enabled status
 *
 * @return 
 * 		void
 *
 */
void setSleepEnabled(unsigned char enabled) {
	writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** 
 * Set memory start address
 * 
 * @param address
 *		address
 *
 * @return 
 * 		void
 *
 */
void setMemoryStartAddress(unsigned char address) {
	writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

/** 
 * read memory byte
 * 
 * @param 
 *		void
 *
 * @return 
 * 		byte
 *
 */
unsigned char readMemoryByte() {
	readByte(devAddr, MPU6050_RA_MEM_R_W, buffer);
	return buffer[0];
}

/** 
 * get offset of x gyro
 * 
 * @param 
 *		void
 *
 * @return 
 * 		offset
 *
 */
char getXGyroOffset() {
	readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}

/** 
 * set offset of x gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setXGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

/** 
 * get offset of y gyro
 * 
 * @param 
 *		void
 *
 * @return 
 * 		offset
 *
 */
char getYGyroOffset() {
	readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}

/** 
 * set offset of y gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setYGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

/** 
 * get offset of z gyro
 * 
 * @param 
 *		void
 *
 * @return 
 * 		offset
 *
 */
char getZGyroOffset() {
	readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}

/** 
 * set offset of z gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setZGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

/** 
 * set user offset of x gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setXGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
}

/** 
 * set user offset of y gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setYGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset);
}

/** 
 * set user offset of z gyro
 * 
 * @param offset
 *		offset
 *
 * @return 
 * 		void
 *
 */
void setZGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset);
}

/** 
 * Set the I2C address of the specified slave (0-3).
 * 
 * @param num 
 *		Slave number (0-3)
 * 
 * @param address 
 *		New address for specified slave
 *
 * @return 
 *		void
 *
 */
void setSlaveAddress(unsigned char num, unsigned char address) {
	if (num > 3)
		return;
	writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, address);
}

/** 
 * Set I2C Master Mode enabled status.
 *
 * @param enabled 
 *		New I2C Master Mode enabled status
 *
 * @return
 *		void	
 * 
 */
void setI2CMasterModeEnabled(unsigned char enabled) {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT,
			enabled);
}

/** 
 * Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 *
 * @param enabled 
 * 		New I2C bypass enabled status
 *
 * @return
 *		void	
 * 
 */
void setI2CBypassEnabled(char enabled) {
	writeBit(devAddr, 0x37, 1, enabled);
}

/** 
 * Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 *
 * @return
 *		void
 *
 */
void resetI2CMaster() {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT,
	true);
}

void setIntEnabled(unsigned char enabled) {
	writeByte(devAddr, MPU6050_RA_INT_ENABLE, enabled);
}

void setRate(unsigned char rate) {
	writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void setDLPFMode(unsigned char mode) {
	writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
	MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void setMotionDetectionThreshold(unsigned char threshold) {
	writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void setZeroMotionDetectionThreshold(unsigned char threshold) {
	writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_MOT_DUR
 */
unsigned char getMotionDetectionDuration() {
	readByte(devAddr, MPU6050_RA_MOT_DUR, buffer);
	return buffer[0];
}
/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void setMotionDetectionDuration(unsigned char duration) {
	writeByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}

/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU6050_RA_ZRMOT_DUR
 */
unsigned char getZeroMotionDetectionDuration() {
	readByte(devAddr, MPU6050_RA_ZRMOT_DUR, buffer);
	return buffer[0];
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void setZeroMotionDetectionDuration(unsigned char duration) {
	writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
unsigned char getIntStatus() {
	readByte(devAddr, MPU6050_RA_INT_STATUS, buffer);
	return buffer[0];
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void setExternalFrameSync(unsigned char sync) {
	writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
	MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void setXGyroOffsetTC(char offset) {
	writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

void setYGyroOffsetTC(char offset) {
	writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

void setZGyroOffsetTC(char offset) {
	writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void reset() {
	writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT,
	true);
}

void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy,
		float* gz) {
	short sax = 0;
	short say = 0;
	short saz = 0;
	short sgx = 0;
	short sgy = 0;
	short sgz = 0;

	getMotion6RawData(&sax, &say, &saz, &sgx, &sgy, &sgz);

	*ax = (float) sax * getAccSensitivityInv();
	*ay = (float) say * getAccSensitivityInv();
	*az = (float) saz * getAccSensitivityInv();
	*gx = (float) sgx * getGyroSensitivityInv() * DE_TO_RA; // rad/sec
	*gy = (float) sgy * getGyroSensitivityInv() * DE_TO_RA; // rad/sec
	*gz = (float) sgz * getGyroSensitivityInv() * DE_TO_RA; // rad/sec
}

#ifdef MPU6050_9AXIS
/** get magnet from AK8975 inside mpu9150/mpu9250 in continous mode 2
 * @param mx magnet of x
 * @param my magnet of y
 * @param mz magnet of z
 */
void getMagnet(short* mx, short* my, short* mz) {
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
	readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 8, buffer);
	*mx = ((((short)buffer[1]) << 8) | buffer[0]);
	*my = ((((short)buffer[3]) << 8) | buffer[2]);
	*mz = ((((short)buffer[5]) << 8) | buffer[4]);
	//_DEBUG(DEBUG_NORMAL,"%d %d %d %d %d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
	//_DEBUG(DEBUG_NORMAL,"RAW mx=%d, my=%d, mz=%d\n",*mx,*my,*mz);
}
#endif

