

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <memory.h>
#include <math.h>

#include "commonLib.h"
#include "i2c.h"
#include "ahrs.h"
#include "mpu6050.h"


#define pgm_read_byte(p) (*(const unsigned char *)(p))
#define AUTO_CAL_BUFFER_SIZE 10
#define AUTO_CAL_GYRO_DEADZONE 0.1
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9150_RA_MAG_ADDRESS		0x0C
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
#define MPU6050_RA_DMP_INT_STATUS   0x39
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
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
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
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0
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
#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
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
#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16
#define MRES  (10.*1229./4096.f) // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale

static unsigned char devAddr;
static unsigned char scaleGyroRange;
static unsigned char scaleAccRange;
static char dmpReady;      // set true if DMP init was successful
static unsigned char buffer[14];
static unsigned char *dmpPacketBuffer;
static unsigned short dmpPacketSize;
static unsigned short packetSize;     // expected DMP packet size (default is 42 bytes)
static unsigned char devStatus; // return status after each device operation (0 = success, !0 = error)
static unsigned char mpuIntStatus;   // holds actual interrupt status byte from MPU
static unsigned char fifoBuffer[64]; // FIFO storage buffer
static short xGyroOffset;  //fot auto calibration
static short yGyroOffset;//fot auto calibration
static short zGyroOffset;//fot auto calibration
static float cal_mean_gx;//fot auto calibration
static float cal_mean_gy;//fot auto calibration
static float cal_mean_gz;//fot auto calibration
static float yaw;
static float pitch;
static float roll;
static float yawGyro;
static float pitchGyro;
static float rollGyro;
static float asaX;
static float asaY;
static float asaZ;

bool testConnection();
unsigned char getDeviceID();
void meansensors();
void getAcceleration(short *x, short *y, short *z);
float convertMpu6050Acceleration(short sampleValue);
unsigned char getClockSource();
void setClockSource(unsigned char source);
unsigned char getFullScaleGyroRange();
void setFullScaleGyroRange(unsigned char range);
unsigned char getFullScaleAccelRange();
void setFullScaleAccelRange(unsigned char range);
unsigned char getSleepEnabled();
void setSleepEnabled(unsigned char enabled);
void setMemoryBank(unsigned char bank, unsigned char prefetchEnabled,
		unsigned char userBank);
void setMemoryStartAddress(unsigned char address);
unsigned char readMemoryByte();
char getXGyroOffset();
void setXGyroOffset(char offset);
char getYGyroOffset();
void setYGyroOffset(char offset);
char getZGyroOffset();
void setZGyroOffset(char offset);
short getXGyroOffsetUser();
void setXGyroOffsetUser(short offset);
short getYGyroOffsetUser();
void setYGyroOffsetUser(short offset);
short getZGyroOffsetUser();
void setZGyroOffsetUser(short offset);
unsigned char getOTPBankValid();
float ak8963SensitivityAdjustment(char asa);
void getMagnet(short* mx, short* my, short* mz);
void setSlaveAddress(unsigned char num, unsigned char address);
void setI2CMasterModeEnabled(unsigned char enabled);
void setI2CBypassEnabled(char enabled);
char getI2CBypassEnabled(void);
unsigned char writeProgDMPConfigurationSet(const unsigned char *data,
		unsigned short dataSize);
unsigned char writeProgMemoryBlock(const unsigned char *data,
		unsigned short dataSize, unsigned char bank, unsigned char address,
		unsigned char verify);
unsigned char writeDMPConfigurationSet(const unsigned char *data,
		unsigned short dataSize, unsigned char useProgMem);
void writeMemoryByte(unsigned char data);
unsigned char writeMemoryBlock(const unsigned char *data,
		unsigned short dataSize, unsigned char bank, unsigned char address,
		unsigned char verify, unsigned char useProgMem);
void setIntEnabled(unsigned char enabled);
void setRate(unsigned char rate);
void setDLPFMode(unsigned char mode);
unsigned char getDMPConfig1();
void setDMPConfig1(unsigned char config);
unsigned char getDMPConfig2();
void setDMPConfig2(unsigned char config);
void setOTPBankValid(unsigned char enabled);
void resetFIFO();
unsigned short getFIFOCount();
unsigned char getFIFOByte();
void getFIFOBytes(unsigned char *data, unsigned char length);
void setFIFOByte(unsigned char data);
unsigned char getMotionDetectionThreshold();
void setMotionDetectionThreshold(unsigned char threshold);
unsigned char getZeroMotionDetectionThreshold();
void setZeroMotionDetectionThreshold(unsigned char threshold);
unsigned char getMotionDetectionDuration();
void setMotionDetectionDuration(unsigned char duration);
unsigned char getZeroMotionDetectionDuration();
void setZeroMotionDetectionDuration(unsigned char duration);
char getFIFOEnabled();
void setFIFOEnabled(char enabled);
char getDMPEnabled();
void setDMPEnabled(char enabled);
void resetDMP();
unsigned char getIntStatus();
void setExternalFrameSync(unsigned char sync);
void getMpu6050Rotation(short int * x, short int * y, short int * z);
short int getRotationX();
short int getRotationY();
short int getRotationZ();
void reset();
char getXGyroOffsetTC();
void setXGyroOffsetTC(char offset);
char getYGyroOffsetTC();
void setYGyroOffsetTC(char offset);
char getZGyroOffsetTC();
void setZGyroOffsetTC(char offset);
unsigned char dmpInitialize();
unsigned short dmpGetFIFOPacketSize();
unsigned char dmpGetYawPitchRoll(float *data, float *q, float *gravity);
unsigned char dmpGetGravity(float *gravity, float *q);
unsigned char dmpGetGyro(short *data, const unsigned char* packet);
unsigned char dmpGetGyro2(float *data, const unsigned char* packet);
unsigned char dmpGetQuaternion(float *q, const unsigned char* packet);
unsigned char sub_dmpGetQuaternion(short *qi, const unsigned char* packet);
unsigned char dmpGetAccel(short *data, const unsigned char* packet);

#ifdef  MPU6050_9AXIS
unsigned char dmpGetMag(short *data, const unsigned char* packet);
//Magnetometer Registers
#define MPU9150_MPU9250_RA_MAG_ADDRESS		0x0C
#define MPU9150_MPU9250_RA_MAG_XOUT_L		0x03
#define MPU9150_MPU9250_RA_MAG_XOUT_H		0x04
#define MPU9150_MPU9250_RA_MAG_YOUT_L		0x05
#define MPU9150_MPU9250_RA_MAG_YOUT_H		0x06
#define MPU9150_MPU9250_RA_MAG_ZOUT_L		0x07
#define MPU9150_MPU9250_RA_MAG_ZOUT_H		0x08
#define MPU9150_MPU9250_RA_INT_PIN_CFG      0x37
#define MPU6050_DMP_CODE_SIZE       1962    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     232     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    140     // dmpUpdates[]

/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
#define prog_uchar unsigned char
#define PROGMEM

const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {
		// bank 0, 256 bytes
		0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02,
		0x00, 0x03, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00,
		0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01, 0x00, 0x1B, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09,
		0x3E, 0x80, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x41, 0xFF, 0x00, 0x00,
		0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
		0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00,
		0x00, 0x05, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
		0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9,
		0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC, 0x32, 0x00, 0x13, 0x9D,
		0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
		0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6,
		0x00, 0x00, 0x27, 0x10,

		// bank 1, 256 bytes
		0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x36,
		0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
		0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2,
		0x00, 0xCE, 0xBB, 0xF7, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04,
		0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C, 0xFF, 0xC2, 0x80, 0x00,
		0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
		0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6,
		0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C, 0x80, 0x00, 0x00, 0x00,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0,
		0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE,
		0x00, 0x0C, 0x02, 0xD0,

		// bank 2, 256 bytes
		0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00,
		0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
		0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x78, 0xA2,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,

		// bank 3, 256 bytes
		0xD8, 0xDC, 0xF4, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xF1, 0xBA, 0xA2,
		0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x98, 0xF7, 0x4A, 0x90, 0x7F, 0x91,
		0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE,
		0x81, 0xF3, 0xC2, 0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80,
		0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF, 0xDF, 0xDF, 0xF2, 0xA7, 0xC3,
		0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C, 0x95,
		0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55,
		0x7D, 0xD8, 0xB1, 0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3,
		0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01, 0xB0, 0x98, 0x87, 0xD9, 0x43,
		0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80, 0xF1,
		0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD,
		0xC7, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89,
		0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80, 0xAC, 0xDE, 0xF2, 0xCA, 0xF1,
		0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E, 0xB8,
		0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56,
		0x76, 0xF1, 0xB9, 0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11,
		0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18,
		0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xF0, 0x00, 0x28,
		0x50, 0xF5, 0xBA, 0xAD, 0x8F, 0x9F, 0x28, 0x54, 0x7C, 0xB9, 0xF1, 0xA3,
		0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xDB, 0xB2, 0xB6, 0x8E, 0x9D,
		0xAE, 0xF5, 0x60, 0x68, 0x70, 0xB1, 0xB5, 0xF1, 0xDA, 0xA6, 0xDF, 0xD9,
		0xA6, 0xFA, 0xA3, 0x86,

		// bank 4, 256 bytes
		0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5,
		0xC7, 0xB2, 0x8C, 0xC1, 0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF,
		0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86, 0xB4, 0x98, 0x0D, 0x35,
		0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
		0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A,
		0xB8, 0xAA, 0x87, 0x2C, 0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3,
		0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8, 0xA9, 0xB4, 0x99, 0x83,
		0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
		0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97,
		0x83, 0xA8, 0x11, 0x84, 0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24,
		0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5, 0x29, 0x55, 0x7D, 0xA5,
		0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
		0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18,
		0x97, 0x82, 0xA8, 0xF1, 0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10,
		0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5, 0x94, 0x01, 0xD9, 0xA3,
		0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
		0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32,
		0xD8, 0x50, 0x71, 0xD9, 0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58,
		0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D, 0xDA, 0x2A, 0xD8, 0x48,
		0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
		0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8,
		0x78, 0xA8, 0x8A, 0x9A,

		// bank 5, 256 bytes
		0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98,
		0xA8, 0xD9, 0x08, 0xD8, 0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA,
		0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87, 0x34, 0xB5, 0xB9, 0x94,
		0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
		0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9,
		0x61, 0xD8, 0x6C, 0x68, 0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9,
		0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D, 0xA3, 0x83, 0x1A, 0x3E,
		0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
		0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8,
		0x87, 0x9A, 0x35, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8,
		0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56, 0xA5, 0x81, 0x00, 0x0C,
		0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
		0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8,
		0xA3, 0x29, 0x83, 0xDA, 0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA,
		0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06,
		0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40,
		0xB9, 0xA3, 0x8A, 0xC3, 0xC5, 0xC7, 0x9A, 0xA3, 0x28, 0x50, 0x78, 0xF1,
		0xB5, 0x93, 0x01, 0xD9, 0xDF, 0xDF, 0xDF, 0xD8, 0xB8, 0xB4, 0xA8, 0x8C,
		0x9C, 0xF0, 0x04, 0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82,
		0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78, 0x78, 0x9B, 0xF1, 0x1A, 0xB0,
		0xF0, 0xB1, 0x83, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0xB0, 0x8B, 0x29, 0x51,
		0x79, 0xB1, 0x83, 0x24,

		// bank 6, 256 bytes
		0x70, 0x59, 0xB0, 0x8B, 0x20, 0x58, 0x71, 0xB1, 0x83, 0x44, 0x69, 0x38,
		0xB0, 0x8B, 0x39, 0x40, 0x68, 0xB1, 0x83, 0x64, 0x48, 0x31, 0xB0, 0x8B,
		0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68, 0x11,
		0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0,
		0x8C, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59,
		0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66, 0xF0, 0x89, 0x9C, 0xA8, 0x29,
		0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xA9,
		0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19,
		0x31, 0x48, 0x60, 0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1,
		0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76, 0x7E, 0xA9, 0x99, 0x88, 0x2D,
		0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56, 0x8A,
		0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E,
		0x9D, 0xB8, 0xAD, 0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99,
		0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91, 0xAC, 0x38, 0xAD, 0x3A, 0xB5,
		0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8, 0x6D,
		0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51,
		0xD9, 0x04, 0xAE, 0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD,
		0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9, 0x01, 0xD8, 0xF2, 0xAE, 0xDA,
		0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD, 0xAD,
		0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76,
		0xF3, 0xAC, 0x2E, 0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28,
		0x28, 0x28, 0x9C, 0xAC,

		// bank 7, 170 bytes (remainder)
		0x30, 0x18, 0xA8, 0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28,
		0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89, 0xAC, 0x91, 0x2C, 0x4C, 0x6C,
		0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79, 0xD9,
		0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D,
		0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1,
		0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA, 0xD8, 0xD8, 0x85, 0x69, 0xDA,
		0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB, 0x8B,
		0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0, 0x87, 0x9C, 0xB9,
		0xA3, 0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3,
		0x9D, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2,
		0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB2, 0xA3,
		0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3,
		0xA3, 0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3,
		0xF3, 0x9B, 0xA3, 0xA3, 0xDC, 0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8,
		0xD8, 0xFF };

#ifndef DMP_FIFO_RATE
// This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
// 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
// DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))
#define DMP_FIFO_RATE	0x03
//0x03
#endif

const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM = {
//  BANK    OFFSET  LENGTH  [DATA]
		0x02, 0xEC, 0x04, 0x00, 0x47, 0x7D, 0x1A,   // ?
		0x03, 0x82, 0x03, 0x4C, 0xCD, 0x6C,   // FCFG_1 inv_set_gyro_calibration
		0x03, 0xB2, 0x03, 0x36, 0x56, 0x76,   // FCFG_3 inv_set_gyro_calibration
		0x00, 0x68, 0x04, 0x02, 0xCA, 0xE3, 0x09, // D_0_104 inv_set_gyro_calibration
		0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, // D_1_152 inv_set_accel_calibration
		0x03, 0x86, 0x03, 0x0C, 0xC9, 0x2C,  // FCFG_2 inv_set_accel_calibration
		0x03, 0x90, 0x03, 0x26, 0x46, 0x66, //   (continued)...FCFG_2 inv_set_accel_calibration
		0x00, 0x6C, 0x02, 0x40, 0x00,       // D_0_108 inv_set_accel_calibration

		0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_00 inv_set_compass_calibration
		0x02, 0x44, 0x04, 0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_01
		0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
		0x02, 0x4C, 0x04, 0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_10
		0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
		0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
		0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
		0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
		0x02, 0xBC, 0x04, 0xC0, 0x00, 0x00, 0x00,   // CPASS_MTX_22

		0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00, // D_1_236 inv_apply_endian_accel
		0x03, 0x86, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
		0x04, 0x22, 0x03, 0x0D, 0x35, 0x5D, // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
		0x00, 0xA3, 0x01, 0x00,                     // ?
		0x04, 0x29, 0x04, 0x87, 0x2D, 0x35, 0x3D,  // FCFG_5 inv_set_bias_update
		0x07, 0x62, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
		0x07, 0x9F, 0x01, 0x30,                     // CFG_16 inv_set_footer
		0x07, 0x67, 0x01, 0x9A,                 // CFG_GYRO_SOURCE inv_send_gyro
		0x07, 0x68, 0x04, 0xF1, 0x28, 0x30, 0x38, // CFG_9 inv_send_gyro -> inv_construct3_fifo
		0x07, 0x62, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38, // ?
		0x02, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00,   // ?
		0x07, 0x83, 0x06, 0xC2, 0xCA, 0xC4, 0xA3, 0xA3, 0xA3, // ?
		// SPECIAL 0x01 = enable interrupts
		0x00, 0x00, 0x00, 0x01, // SET INT_ENABLE, SPECIAL INSTRUCTION
		0x07, 0xA7, 0x01, 0xFE,                     // ?
		0x07, 0x62, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38, // ?
		0x07, 0x67, 0x01, 0x9A,                     // ?
		0x07, 0x68, 0x04, 0xF1, 0x28, 0x30, 0x38, // CFG_12 inv_send_accel -> inv_construct3_fifo
		0x07, 0x8D, 0x04, 0xF1, 0x28, 0x30, 0x38, // ??? CFG_12 inv_send_mag -> inv_construct3_fifo
		0x02, 0x16, 0x02, 0x00, DMP_FIFO_RATE       // D_0_22 inv_set_fifo_rate

		// It is important to make sure the host processor can keep up with reading and processing
		// the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
		};

const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM = { 0x01, 0xB2,
		0x02, 0xFF, 0xF5, 0x01, 0x90, 0x04, 0x0A, 0x0D, 0x97, 0xC0, 0x00, 0xA3,
		0x01, 0x00, 0x04, 0x29, 0x04, 0x87, 0x2D, 0x35, 0x3D, 0x01, 0x6A, 0x02,
		0x06, 0x00, 0x01, 0x60, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x60, 0x04, 0x40, 0x00, 0x00, 0x00, 0x02, 0x60, 0x0C, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x08, 0x02, 0x01, 0x20, 0x01, 0x0A, 0x02, 0x00, 0x4E, 0x01, 0x02, 0x02,
		0xFE, 0xB3, 0x02, 0x6C, 0x04, 0x00, 0x00, 0x00,
		0x00, // READ
		0x02, 0x6C, 0x04, 0xFA, 0xFE, 0x00, 0x00, 0x02, 0x60, 0x0C, 0xFF, 0xFF,
		0xCB, 0x4D, 0x00, 0x01, 0x08, 0xC1, 0xFF, 0xFF, 0xBC, 0x2C, 0x02, 0xF4,
		0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0xF8, 0x04, 0x00, 0x00, 0x00, 0x00,
		0x02, 0xFC, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x04, 0x40, 0x00,
		0x00, 0x00, 0x00, 0x60, 0x04, 0x00, 0x40, 0x00, 0x00 };



#else

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

#define prog_uchar unsigned char
#define PROGMEM

const prog_uchar dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {
	// bank 0, 256 bytes
	0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
	0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
	0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
	0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
	0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

	// bank 1, 256 bytes
	0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
	0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
	0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
	0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

	// bank 2, 256 bytes
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// bank 3, 256 bytes
	0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
	0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
	0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
	0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
	0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
	0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
	0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
	0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
	0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
	0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
	0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
	0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
	0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
	0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
	0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
	0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

	// bank 4, 256 bytes
	0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
	0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
	0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
	0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
	0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
	0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
	0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
	0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
	0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
	0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
	0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
	0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

	// bank 5, 256 bytes
	0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
	0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
	0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
	0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
	0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
	0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
	0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
	0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
	0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
	0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
	0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
	0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
	0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
	0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
	0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
	0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

	// bank 6, 256 bytes
	0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
	0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
	0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
	0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
	0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
	0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
	0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
	0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
	0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
	0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
	0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
	0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
	0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
	0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
	0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
	0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

	// bank 7, 138 bytes (remainder)
	0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
	0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
	0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
	0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
	0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
	0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
	0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
	0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
	0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

// DMP FIFO update rate: 0x09 drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
// 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very
// noisy data.  DMP output frequency is calculated easily using this equation:
// (200Hz / (1 + value))

// It is important to make sure the host processor can keep up with reading and
// processing the FIFO output at the desired rate. Handling FIFO overflow
// cleanly is also a good idea.  thanks to Noah Zerkin for piecing this stuff
// together!

#ifndef DMP_FIFO_RATE
//200/(1+DMP_FIFO_RATE) HZ
#define DMP_FIFO_RATE	0x02
#endif

const prog_uchar dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM = {
//  BANK    OFFSET  LENGTH  [DATA]
	0x03, 0x7B, 0x03, 0x4C, 0xCD, 0x6C,// FCFG_1 inv_set_gyro_calibration
	0x03, 0xAB, 0x03, 0x36, 0x56, 0x76,// FCFG_3 inv_set_gyro_calibration
	0x00, 0x68, 0x04, 0x02, 0xCB, 0x47, 0xA2,// D_0_104 inv_set_gyro_calibration
	0x02, 0x18, 0x04, 0x00, 0x05, 0x8B, 0xC1,// D_0_24 inv_set_gyro_calibration
	0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00,// D_1_152 inv_set_accel_calibration
	0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,// FCFG_2 inv_set_accel_calibration
	0x03, 0x89, 0x03, 0x26, 0x46, 0x66,// FCFG_7 inv_set_accel_calibration
	0x00, 0x6C, 0x02, 0x20, 0x00,// D_0_108 inv_set_accel_calibration
	0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_00 inv_set_compass_calibration
	0x02, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_01
	0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_02
	0x02, 0x4C, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_10
	0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_11
	0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_12
	0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_20
	0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_21
	0x02, 0xBC, 0x04, 0x00, 0x00, 0x00, 0x00,// CPASS_MTX_22
	0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00,// D_1_236 inv_apply_endian_accel
	0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,// FCFG_2 inv_set_mpu_sensors
	0x04, 0x02, 0x03, 0x0D, 0x35, 0x5D,// CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
	0x04, 0x09, 0x04, 0x87, 0x2D, 0x35, 0x3D,// FCFG_5 inv_set_bias_update
	0x00, 0xA3, 0x01, 0x00,// D_0_163 inv_set_dead_zone
	// SPECIAL 0x01 = enable interrupts
	0x00, 0x00, 0x00, 0x01,// SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
	0x07, 0x86, 0x01, 0xFE,// CFG_6 inv_set_fifo_interupt
	0x07, 0x41, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38,// CFG_8 inv_send_quaternion
	0x07, 0x7E, 0x01, 0x30,// CFG_16 inv_set_footer
	0x07, 0x46, 0x01, 0x9A,// CFG_GYRO_SOURCE inv_send_gyro
	0x07, 0x47, 0x04, 0xF1, 0x28, 0x30, 0x38,// CFG_9 inv_send_gyro -> inv_construct3_fifo
	0x07, 0x6C, 0x04, 0xF1, 0x28, 0x30, 0x38,// CFG_12 inv_send_accel -> inv_construct3_fifo
	0x02, 0x16, 0x02, 0x00, DMP_FIFO_RATE// D_0_22 inv_set_fifo_rate
};

const prog_uchar dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM = {
	0x01, 0xB2, 0x02, 0xFF, 0xFF,
	0x01, 0x90, 0x04, 0x09, 0x23, 0xA1, 0x35,
	0x01, 0x6A, 0x02, 0x06, 0x00,
	0x01, 0x60, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x60, 0x04, 0x40, 0x00, 0x00, 0x00,
	0x01, 0x62, 0x02, 0x00, 0x00,
	0x00, 0x60, 0x04, 0x00, 0x40, 0x00, 0x00
};

#endif


bool mpu6050Init() {

	if(checkI2cDeviceIsExist(MPU6050_ADDRESS)){
		_DEBUG(DEBUG_NORMAL,"MPU6050 exist\n");
	}else{
		_DEBUG(DEBUG_NORMAL,"MPU6050 dowsn't exist\n");
		return false;
	}
	
	devAddr = MPU6050_DEFAULT_ADDRESS;
	scaleGyroRange=0;
	scaleAccRange=0;
	xGyroOffset=12;  //pitch
	yGyroOffset=-46;  // row
	zGyroOffset=-10; //row

#if  defined(MPU_DMP) || defined(MPU_DMP_YAW)	
	dmpReady = false;
	setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	usleep(1000);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
	usleep(1000);

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		printf("Enabling DMP...\n");
		setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d: %s)\n", devStatus,
				devStatus == 1 ?
						"initial memory load failed" :
						"DMP configuration updates failed");
		return false;

	}
#else

		writeByte(devAddr, 0x6B, 0x08);// PWR_MGMT_1 Reset Device
		usleep(1000);
		setSleepEnabled(false);
		usleep(1000);
		writeByte(devAddr, 0x6B, 0x01);// PWR_MGMT_1 Clock Source
		usleep(1000);
		printf("Setting DLPF bandwidth to 20Hz...\n");
		setDLPFMode(MPU6050_DLPF_BW_20);
		usleep(1000);
		setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
		usleep(1000);
		//printf("Setting sample rate to 200Hz...\n");
        // setRate(4); // 1khz / (1 + 4) = 200 Hz
        usleep(1000);
		setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
		usleep(1000);
    	setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
		usleep(1000);
		setXGyroOffsetUser(0);
		usleep(1000);
		setYGyroOffsetUser(0);
		usleep(1000);
		setZGyroOffsetUser(0);
		usleep(1000);
		writeByte(devAddr, 0x6A, 0x20);// Set I2C_MST_EN
		usleep(1000);
		writeByte(devAddr, 0x37, 0x10);// Set INT_ANYRD_2CLEAR
		usleep(1000);
		writeByte(devAddr, 0x24, 0x4D);// I2C_MST_CTRL, I2C Speed 400 kHz
		usleep(1000);

		//set AK8963
		setI2CMasterModeEnabled(false);
		usleep(1000);
    	setI2CBypassEnabled(true);
		usleep(10000);
		writeByte(MPU9150_RA_MAG_ADDRESS, 0x0B, 0x01);// Reset Device
		usleep(10000);
		writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x0f); // Fuse ROM access mode
		usleep(10000);		
		readBytes(MPU9150_RA_MAG_ADDRESS, 0x10, 3, buffer); //read AK8963 Sensitivity Adjustment values
		asaX=ak8963SensitivityAdjustment(buffer[0]);
		asaY=ak8963SensitivityAdjustment(buffer[1]);
		asaZ=ak8963SensitivityAdjustment(buffer[2]);
		printf("AK8963 Sensitivity Adjustment values: x=%f, y=%f, z=%f\n",asaX,asaY,asaZ);
		usleep(10000);
		writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x00); // Power-down mode
		usleep(10000);
		writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x06|0x10); // Continuous measurement mode 2 (100Hz)|Full Scale
		//writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x02|0x10); // Continuous measurement mode 1 (8Hz)|Full Scale
		usleep(10000);
		setI2CMasterModeEnabled(true);
		usleep(1000);
    	setI2CBypassEnabled(false);
		usleep(1000);
		writeByte(devAddr,0x26, 0x02);//I2C_SLV0_REG(0x26)
		usleep(1000);
		writeByte(devAddr,0x25, MPU9150_RA_MAG_ADDRESS| 0x80);//I2C_SLV0_ADDR(0x25)
		usleep(1000);	
		writeByte(devAddr,0x27, 0x80 | 8);//I2C_SLV0_CTRL
#endif

	usleep(100000);
	return true;
	
}



float getGyroSensitivity(){
	switch(scaleGyroRange){
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
			_ERROR("(%s-%d) Unknow Gyro index\n",__func__,__LINE__);
			return -1.f;
	}
}

float getAccSensitivity(){
	switch(scaleAccRange){
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
			_ERROR("(%s-%d) Unknow acc index\n",__func__,__LINE__);
			return -1.f;
	}
}

float getGyroSensitivityInv(){
	switch(scaleGyroRange){
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
			_ERROR("(%s-%d) Unknow Gyro index\n",__func__,__LINE__);
			return -1.f;
	}
}

float getAccSensitivityInv(){

	switch(scaleAccRange){
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
			_ERROR("(%s-%d) Unknow acc index\n",__func__,__LINE__);
			return -1.f;
	}
}

void meansensors() {
	long i = 0;
	float buff_gx = 0, buff_gy = 0, buff_gz = 0;
	short ax_, ay_, az_, gx_, gy_, gz_;
	float gx, gy, gz;

	while (i < (AUTO_CAL_BUFFER_SIZE + 1)) {
		// read raw gyro measurements from device
		getMotion6RawData(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
		gx = (gx_ *getGyroSensitivityInv());
		gy = (gy_ *getGyroSensitivityInv());
		gz = (gz_ *getGyroSensitivityInv());
		//printf("gx=%2.4f gy=%2.4f gz=%2.4f\n",gx,gy,gz);
		if (i <= (AUTO_CAL_BUFFER_SIZE)) {
			buff_gx = buff_gx + gx;
			buff_gy = buff_gy + gy;
			buff_gz = buff_gz + gz;
		}
		if (i == (AUTO_CAL_BUFFER_SIZE)) {
			cal_mean_gx = (float) ((short) ((buff_gx / AUTO_CAL_BUFFER_SIZE)
					* 10.0)) / 10.0;
			cal_mean_gy = (float) ((short) ((buff_gy / AUTO_CAL_BUFFER_SIZE)
					* 10.0)) / 10.0;
			cal_mean_gz = (float) ((short) ((buff_gz / AUTO_CAL_BUFFER_SIZE)
					* 10.0)) / 10.0;
		}
		i++;
		usleep(5000); //Needed so we don't get repeated measures
	}
	printf("mean gx=%2.4f, gy=%2.4f gz=%2.4f\n", cal_mean_gx, cal_mean_gy,
			cal_mean_gz);

}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool testConnection() {
	return ((getDeviceID()> 0)?true:false);
}

unsigned char getDeviceID() {
	readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT,
	MPU6050_WHO_AM_I_LENGTH, buffer);
	return buffer[0];
}

void setYaw(float t_yaw) {
	yaw = t_yaw;
}

void setPitch(float t_pitch) {
	pitch = t_pitch;
}

void setRoll(float t_roll) {
	roll = t_roll;
}

float getYaw() {
	return yaw;
}

float getPitch() {
	return pitch;
}

float getRoll() {
	return roll;
}

void setYawGyro(float t_yaw_gyro) {
	yawGyro = t_yaw_gyro;
}

void setPitchGyro(float t_pitch_gyro) {
	pitchGyro = t_pitch_gyro;
}

void setRollGyro(float t_roll_gyro) {
	rollGyro = t_roll_gyro;
}

float getYawGyro() {
	return yawGyro;
}

float getPitchGyro() {
	return pitchGyro;
}

float getRollGyro() {
	return rollGyro;
}


// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6RawData()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void getMpu6050Rotation(short int * x, short int * y, short int * z) {
	readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
	*x = (((short int) buffer[0]) << 8) | buffer[1];
	*y = (((short int) buffer[2]) << 8) | buffer[3];
	*z = (((short int) buffer[4]) << 8) | buffer[5];
//   printf("raw gyro: x=%d, y=%d, z=%d \n",*x,*y,*z);
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6RawData()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
short int getRotationX() {
	readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
	return (((short int) buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6RawData()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
short int getRotationY() {
	readBytes(devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
	return (((short int) buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6RawData()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
short int getRotationZ() {
	readBytes(devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
	return (((short int) buffer[0]) << 8) | buffer[1];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */

void getAcceleration(short *x, short *y, short *z) {
	readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
	*x = (((short) buffer[0]) << 8) | buffer[1];
	*y = (((short) buffer[2]) << 8) | buffer[3];
	*z = (((short) buffer[4]) << 8) | buffer[5];
}

/*
 void getAcceleration(float *x, float *y, float *z) {
 readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
 *x = (((short )buffer[0]) << 8) | buffer[1];
 *y = (((short )buffer[2]) << 8) | buffer[3];
 *z = (((short )buffer[4]) << 8) | buffer[5];


 *x = convertMpu6050Acceleration((((short )buffer[0]) << 8) | buffer[1]);
 *y = convertMpu6050Acceleration((((short )buffer[2]) << 8) | buffer[3]);
 *z = convertMpu6050Acceleration((((short )buffer[4]) << 8) | buffer[5]);

 }
 */

float convertMpu6050Acceleration(short sampleValue) {
	float value = ((float) sampleValue
			/ (float) ((float) 16384 / (float) (1 << getFullScaleAccelRange())));
//	printf("%s %d: sampleValue=%d\n",__func__,__LINE__, sampleValue);
	//	printf("%s %d: return value=%2.2f\n",__func__,__LINE__,value);
	return value;
}

/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
unsigned char getClockSource() {
	readBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
	MPU6050_PWR1_CLKSEL_LENGTH, buffer);
	return buffer[0];
}

/** Set clock source setting.
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
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void setClockSource(unsigned char source) {
	writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
	MPU6050_PWR1_CLKSEL_LENGTH, source);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
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
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
unsigned char getFullScaleGyroRange() {
	readBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
	MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
	return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void setFullScaleGyroRange(unsigned char range) {
	writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
	MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	scaleGyroRange=range;
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
unsigned char getFullScaleAccelRange() {
	readBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
	MPU6050_ACONFIG_AFS_SEL_LENGTH, buffer);
	return buffer[0];
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void setFullScaleAccelRange(unsigned char range) {
	writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
	MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	scaleAccRange=range;
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
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

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
unsigned char getSleepEnabled() {
	readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, buffer);
	return buffer[0];
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void setSleepEnabled(unsigned char enabled) {
	writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

// BANK_SEL register

void setMemoryBank(unsigned char bank, unsigned char prefetchEnabled,
		unsigned char userBank) {
	bank &= 0x1F;
	if (userBank)
		bank |= 0x20;
	if (prefetchEnabled)
		bank |= 0x40;
	writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}
// MEM_START_ADDR register

void setMemoryStartAddress(unsigned char address) {
	writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register
unsigned char readMemoryByte() {
	readByte(devAddr, MPU6050_RA_MEM_R_W, buffer);
	return buffer[0];
}
char getXGyroOffset() {
	readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}

void setXGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register
char getYGyroOffset() {
	readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}

void setYGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
char getZGyroOffset() {
	readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setZGyroOffset(char offset) {
	writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

// XG_OFFS_USR* registers

short getXGyroOffsetUser() {
	readBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
	return (((short) buffer[0]) << 8) | buffer[1];
}
void setXGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

short getYGyroOffsetUser() {
	readBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
	return (((short) buffer[0]) << 8) | buffer[1];
}
void setYGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

short getZGyroOffsetUser() {
	readBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
	return (((short) buffer[0]) << 8) | buffer[1];
}
void setZGyroOffsetUser(short offset) {
	writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset);
}

// XG_OFFS_TC register
unsigned char getOTPBankValid() {
	readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
	return buffer[0];
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void setSlaveAddress(unsigned char num, unsigned char address) {
	if (num > 3)
		return;
	writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, address);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void setI2CMasterModeEnabled(unsigned char enabled) {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT,
			enabled);
}

/** Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @return Current I2C bypass enabled status
 * @see MPU9150_RA_INT_PIN_CFG
 * @see MPU9150_INTCFG_I2C_BYPASS_EN_BIT
 */
char getI2CBypassEnabled() {
    readBit(devAddr, 0x37, 1, buffer);
    return buffer[0];
}
/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU9150_RA_INT_PIN_CFG
 * @see MPU9150_INTCFG_I2C_BYPASS_EN_BIT
 */
void setI2CBypassEnabled(char enabled) {
    writeBit(devAddr, 0x37, 1, enabled);
}


/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void resetI2CMaster() {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT,
	true);
}

unsigned char writeProgMemoryBlock(const unsigned char *data,
		unsigned short dataSize, unsigned char bank, unsigned char address,
		unsigned char verify) {
	return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}

unsigned char writeProgDMPConfigurationSet(const unsigned char *data,
		unsigned short dataSize) {
	return writeDMPConfigurationSet(data, dataSize, true);
}

unsigned char writeDMPConfigurationSet(const unsigned char *data,
		unsigned short dataSize, unsigned char useProgMem) {
	unsigned char *progBuffer = NULL, success, special;
	unsigned short i, j;
	if (useProgMem) {
		progBuffer = (unsigned char *) malloc(8); // assume 8-byte blocks, realloc later if necessary
	}

	// config set data is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	unsigned char bank, offset, length;
	for (i = 0; i < dataSize;) {
		if (useProgMem) {
			bank = pgm_read_byte(data + i++);
			offset = pgm_read_byte(data + i++);
			length = pgm_read_byte(data + i++);
		} else {
			bank = data[i++];
			offset = data[i++];
			length = data[i++];
		}

		// write data or perform special action
		if (length > 0) {
			// regular block of data to write
			/*Serial.print("Writing config block to bank ");
			 Serial.print(bank);
			 Serial.print(", offset ");
			 Serial.print(offset);
			 Serial.print(", length=");
			 Serial.println(length);*/
			if (useProgMem) {
				if (sizeof(progBuffer) < length)
					progBuffer = (unsigned char *) realloc(progBuffer, length);
				for (j = 0; j < length; j++)
					progBuffer[j] = pgm_read_byte(data + i + j);
			} else {
				progBuffer = (unsigned char *) data + i;
			}
			success = writeMemoryBlock(progBuffer, length, bank, offset, true,
			false);
			i += length;
		} else {
			// special instruction
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			if (useProgMem) {
				special = pgm_read_byte(data + i++);
			} else {
				special = data[i++];
			}
			/*Serial.print("Special command code ");
			 Serial.print(special, HEX);
			 Serial.println(" found...");*/
			if (special == 0x01) {
				// enable DMP-related interrupts

				//setIntZeroMotionEnabled(true);
				//setIntFIFOBufferOverflowEnabled(true);
				//setIntDMPEnabled(true);
				writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32); // single operation

				success = true;
			} else {
				// unknown special command
				success = false;
			}
		}

		if (!success) {
			if (useProgMem)
				free(progBuffer);
			return false; // uh oh
		}
	}
	if (useProgMem)
		free(progBuffer);
	return true;
}

void writeMemoryByte(unsigned char data) {
	writeByte(devAddr, MPU6050_RA_MEM_R_W, data);
}
void readMemoryBlock(unsigned char *data, unsigned short dataSize,
		unsigned char bank, unsigned char address) {
	setMemoryBank(bank, false, false);
	setMemoryStartAddress(address);
	unsigned char chunkSize;
	unsigned short i;
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize)
			chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address)
			chunkSize = 256 - address;

		// read the chunk of data as specified
		readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0)
				bank++;
			setMemoryBank(bank, false, false);
			setMemoryStartAddress(address);
		}
	}
}

unsigned char writeMemoryBlock(const unsigned char *data,
		unsigned short dataSize, unsigned char bank, unsigned char address,
		unsigned char verify, unsigned char useProgMem) {
	setMemoryBank(bank, false, false);
	setMemoryStartAddress(address);
	unsigned char chunkSize;
	unsigned char *verifyBuffer;
	unsigned char *progBuffer = NULL; // Keep compiler quiet
	unsigned short i;
	unsigned char j;
	if (verify)
		verifyBuffer = (unsigned char *) malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
	if (useProgMem)
		progBuffer = (unsigned char *) malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize)
			chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address)
			chunkSize = 256 - address;

		if (useProgMem) {
			// write the chunk of data as specified
			for (j = 0; j < chunkSize; j++)
				progBuffer[j] = pgm_read_byte(data + i + j);
		} else {
			// write the chunk of data as specified
			progBuffer = (unsigned char *) data + i;
		}

		writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

		// verify data if needed
		if (verify && verifyBuffer) {
			setMemoryBank(bank, false, false);
			setMemoryStartAddress(address);
			readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
			if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
				/*Serial.print("Block write verification error, bank ");
				 Serial.print(bank, DEC);
				 Serial.print(", address ");
				 Serial.print(address, DEC);
				 Serial.print("!\nExpected:");
				 for (j = 0; j < chunkSize; j++) {
				 Serial.print(" 0x");
				 if (progBuffer[j] < 16) Serial.print("0");
				 Serial.print(progBuffer[j], HEX);
				 }
				 Serial.print("\nReceived:");
				 for (uint8_t j = 0; j < chunkSize; j++) {
				 Serial.print(" 0x");
				 if (verifyBuffer[i + j] < 16) Serial.print("0");
				 Serial.print(verifyBuffer[i + j], HEX);
				 }
				 Serial.print("\n");*/
				free(verifyBuffer);
				if (useProgMem)
					free(progBuffer);
				return false; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0)
				bank++;
			setMemoryBank(bank, false, false);
			setMemoryStartAddress(address);
		}
	}
	if (verify)
		free(verifyBuffer);
	if (useProgMem)
		free(progBuffer);
	return true;
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

// DMP_CFG_1 register

unsigned char getDMPConfig1() {
	readByte(devAddr, MPU6050_RA_DMP_CFG_1, buffer);
	return buffer[0];
}
void setDMPConfig1(unsigned char config) {
	writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register

unsigned char getDMPConfig2() {
	readByte(devAddr, MPU6050_RA_DMP_CFG_2, buffer);
	return buffer[0];
}
void setDMPConfig2(unsigned char config) {
	writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config);
}

void setOTPBankValid(unsigned char enabled) {
	writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT,
			enabled);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void resetFIFO() {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT,
	true);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
unsigned short getFIFOCount() {
	readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
	return (((unsigned short) buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
unsigned char getFIFOByte() {
	readByte(devAddr, MPU6050_RA_FIFO_R_W, buffer);
	return buffer[0];
}
void getFIFOBytes(unsigned char *data, unsigned char length) {
	readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data);
}
/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU6050_RA_FIFO_R_W
 */
void setFIFOByte(unsigned char data) {
	writeByte(devAddr, MPU6050_RA_FIFO_R_W, data);
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void setMotionDetectionThreshold(unsigned char threshold) {
	writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_MOT_THR
 */
unsigned char getMotionDetectionThreshold() {
	readByte(devAddr, MPU6050_RA_MOT_THR, buffer);
	return buffer[0];
}

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_ZRMOT_THR
 */
unsigned char getZeroMotionDetectionThreshold() {
	readByte(devAddr, MPU6050_RA_ZRMOT_THR, buffer);
	return buffer[0];
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

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
char getFIFOEnabled() {
	readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,
			buffer);
	return buffer[0];
}
/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void setFIFOEnabled(char enabled) {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,
			enabled);
}

char getDMPEnabled() {
	readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, buffer);
	return buffer[0];
}

void setDMPEnabled(char enabled) {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,
			enabled);
}

void resetDMP() {
	writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT,
	true);
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

char getXGyroOffsetTC() {
	readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setXGyroOffsetTC(char offset) {
	writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register

char getYGyroOffsetTC() {
	readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setYGyroOffsetTC(char offset) {
	writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

char getZGyroOffsetTC() {
	readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
	MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
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

unsigned char getYawPitchRollInfo(float *yprAttitude, float *yprRate, float *xyzAcc, 
		float *xyzGravity, float *xyzMagnet) {

	float q[4];		    // [w, x, y, z]         quaternion container
	float gravity[3];   // [x, y, z]            gravity 
	unsigned char	result=0;
 
#if defined(MPU_DMP)|| defined(MPU_DMP_YAW)

	short acc[3];  		// [x, y, z] acc
	short rate[3];  	// [x, y, z] rate
	unsigned short fifoCount = 0;

	memset(q, 0, sizeof(q));
	memset(gravity, 0, sizeof(gravity));
	memset(acc, 0, sizeof(acc));
	memset(rate, 0, sizeof(rate));
	memset(yprAttitude, 0, sizeof(float)*3);

	// if programming failed, don't try to do anything
	if (!dmpReady)
		return false;

	// get current FIFO count
	fifoCount = getFIFOCount();

	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		resetFIFO();
		result=1;
	} else if (fifoCount >= dmpPacketSize) {

		getFIFOBytes(fifoBuffer, packetSize);
		dmpGetQuaternion(q, fifoBuffer);
		dmpGetGravity(gravity, q);
		dmpGetYawPitchRoll(yprAttitude, q, gravity);

#ifndef MPU_DMP_YAW
		dmpGetGyro(rate, fifoBuffer);
		dmpGetAccel(acc, fifoBuffer);
		xyzGravity[0]=gravity[0];
		xyzGravity[1]=gravity[1];
		xyzGravity[2]=gravity[2];
		yprRate[0]=(float)rate[0];
		yprRate[1]=(float)rate[1];
		yprRate[2]=(float)rate[2];
		xyzAcc[0]=(float)acc[0]*getAccSensitivityInv();
		xyzAcc[1]=(float)acc[1]*getAccSensitivityInv();
		xyzAcc[2]=(float)acc[2]*getAccSensitivityInv();
		yprAttitude[1] = yprAttitude[1] * RA_TO_DE;
		yprAttitude[2] = yprAttitude[2] * RA_TO_DE;
#endif
		yprAttitude[0] = yprAttitude[0] * RA_TO_DE;
		result=0;
	} else {
		result=2;
	}

#ifdef MPU_DMP_YAW
	float ax=0.f;
	float ay=0.f;
	float az=0.f;
	float gx=0.f;
	float gy=0.f;
	float gz=0.f;
	float yawtmp=0.f;
	
	if(0==result){
		yawtmp=yprAttitude[0];
	}
	
	getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	IMUupdate(gx, gy,gz,ax,ay,az,q);
    dmpGetGravity(gravity, q);
	dmpGetYawPitchRoll(yprAttitude, q, gravity);
    yprAttitude[1] = yprAttitude[1] * RA_TO_DE;
    yprAttitude[2] = yprAttitude[2] * RA_TO_DE;
	if(0==result){
		yprAttitude[0]=yawtmp;
	}
	xyzGravity[0]=gravity[0];
	xyzGravity[1]=gravity[1];
	xyzGravity[2]=gravity[2];
	yprRate[0]=gx*  RA_TO_DE;
	yprRate[1]=gy*  RA_TO_DE;
	yprRate[2]=gz*  RA_TO_DE;
	xyzAcc[0]=ax;
	xyzAcc[1]=ay;
	xyzAcc[2]=az;
#endif
	return result;
#else
		float ax=0;
		float ay=0;
		float az=0;
		float gx=0;
		float gy=0;
		float gz=0;

		getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		IMUupdate(gx, gy,gz,ax,ay,az,q);
        dmpGetGravity(gravity, q);
		dmpGetYawPitchRoll(yprAttitude, q, gravity);

	    yprAttitude[0] = yprAttitude[0] * RA_TO_DE;
        yprAttitude[1] = yprAttitude[1] * RA_TO_DE;
        yprAttitude[2] = yprAttitude[2] * RA_TO_DE;
		xyzGravity[0]=gravity[0];
		xyzGravity[1]=gravity[1];
		xyzGravity[2]=gravity[2];
		yprRate[0]=gx* RA_TO_DE;
		yprRate[1]=gy* RA_TO_DE;
		yprRate[2]=gz* RA_TO_DE;
		xyzAcc[0]=ax;
		xyzAcc[1]=ay;
		xyzAcc[2]=az;
		xyzMagnet[0]=0.f;
		xyzMagnet[1]=0.f;
		xyzMagnet[2]=0.f;
		return 0;
#endif
}

/*FOR MCU6550 DMP*/

unsigned short dmpGetFIFOPacketSize() {
	return dmpPacketSize;
}

unsigned char dmpGetYawPitchRoll(float *data, float *q, float *gravity) {
	// yaw: (about Z axis)
	data[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(
			gravity[0]
					/ sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(
			gravity[1]
					/ sqrt(gravity[0] * gravity[0] + gravity[2] * gravity[2]));
	return 0;
}

unsigned char dmpGetGravity(float *gravity, float *q) {
	gravity[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	gravity[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
	gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	return 0;
}

unsigned char dmpGetGyro2(float *data, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (float) (((((unsigned int) packet[16] << 24)
			| ((unsigned int) packet[17] << 16)
			| ((unsigned int) packet[18] << 8) | packet[19])));
	data[1] = (float) (((((unsigned int) packet[20] << 24)
			| ((unsigned int) packet[21] << 16)
			| ((unsigned int) packet[22] << 8) | packet[23])));
	data[2] = (float) (((((unsigned int) packet[24] << 24)
			| ((unsigned int) packet[25] << 16)
			| ((unsigned int) packet[26] << 8) | packet[27])));
	return 0;
}
unsigned char dmpGetGyro(short *data, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = ((packet[16] << 8) | packet[17]);
	data[1] = ((packet[20] << 8) | packet[21]);
	data[2] = ((packet[24] << 8) | packet[25]);
	//printf("data[0]=%3.3f, data[1]=%3.3f, data[2]=%3.3f\n",data[0],data[1],data[2]);
	return 0;
}

unsigned char sub_dmpGetQuaternion(short *qi, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	qi[0] = ((packet[0] << 8) + packet[1]);
	qi[1] = ((packet[4] << 8) + packet[5]);
	qi[2] = ((packet[8] << 8) + packet[9]);
	qi[3] = ((packet[12] << 8) + packet[13]);
	return 0;
}

unsigned char dmpGetQuaternion(float *q, const unsigned char* packet) {
	short qI[4];
	unsigned char status = sub_dmpGetQuaternion(qI, packet);
	if (status == 0) {
		q[0] = (float) qI[0] / 16384.0f;
		q[1] = (float) qI[1] / 16384.0f;
		q[2] = (float) qI[2] / 16384.0f;
		q[3] = (float) qI[3] / 16384.0f;
		return 0;
	}
	return status; // int16 return value, indicates error if this line is reached

}

#ifdef MPU6050_9AXIS

/** get magnet from AK8975 inside mpu9150/mpu9250 in continous mode 2
* @param mx magnet of x
* @param my magnet of y
* @param mz magnet of z
*/
void getMagnet(short* mx, short* my, short* mz){
#if 1

	readBytes(devAddr, 0x49, 8, buffer); //I2C_MST_EN
	*mx = ((((short)buffer[2]) << 8) | buffer[1]);
    *my = ((((short)buffer[4]) << 8) | buffer[3]);
    *mz = ((((short)buffer[6]) << 8) | buffer[5]);
	//printf("mx=%d, my=%d, mz=%d\n",*mx,*my,*mz);

	//printf("%d %d %d %d %d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
	#else
    writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
    usleep(10000);
    readBytes(MPU9150_RA_MAG_ADDRESS, 0x02, 8, buffer);
	*mx = (((short)buffer[1]) << 8) | buffer[2];
    *my = (((short)buffer[3]) << 8) | buffer[4];
    *mz = (((short)buffer[5]) << 8) | buffer[6];
	printf("%d %d %d %d %d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
	//printf("mx=%d, my=%d, mz=%d\n",*mx,*my,*mz);
#endif	
}


float ak8963SensitivityAdjustment(char asa){
	return ((((float)asa-128.f)*0.5)/128.f)+1.f;
}


void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){
	short sax=0;
    short say=0;
    short saz=0;
    short sgx=0;
    short sgy=0;
    short sgz=0;;

	getMotion6RawData(&sax, &say, &saz, &sgx, &sgy, &sgz);

	*ax=(float)sax*getAccSensitivityInv();
	*ay=(float)say*getAccSensitivityInv();
	*az=(float)saz*getAccSensitivityInv();
	*gx=(float)sgx*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	*gy=(float)sgy*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	*gz=(float)sgz*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	
}

void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz,float* mx, float* my, float* mz){
	short sax=0;
    short say=0;
    short saz=0;
    short sgx=0;
    short sgy=0;
    short sgz=0;
    short smx=0;
    short smy=0;
    short smz=0;

	getMotion6RawData(&sax, &say, &saz, &sgx, &sgy, &sgz);
	getMagnet(&smx, &smy, &smz);

	*ax=(float)sax*getAccSensitivityInv();
	*ay=(float)say*getAccSensitivityInv();
	*az=(float)saz*getAccSensitivityInv();
	*gx=(float)sgx*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	*gy=(float)sgy*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	*gz=(float)sgz*getGyroSensitivityInv()* DE_TO_RA; // rad/sec
	*mx=(float)smx*0.15*asaX; //uT
    *my=(float)smy*0.15*asaY;  //uT
    *mz=(float)smz*0.15*asaZ;  //uT
	
}


unsigned char dmpInitialize() {
	
	// reset device
	printf("\n\nResetting MPU6050 9 AXIS ...\n");
	reset();
	usleep(120000); // wait after reset

	// disable sleep mode
	printf("Disabling sleep mode...\n");
	setSleepEnabled(false);

	// get MPU product ID
	// printf("Getting product ID...\n");
	//uint8_t productID = 0; //getProductID();
	// printf("Product ID = ");
	//printf(productID);

	// get MPU hardware revision
	printf("Selecting user bank 16...\n");
	setMemoryBank(0x10, true, true);
	printf("Selecting memory byte 6...\n");
	setMemoryStartAddress(0x06);
	printf("Checking hardware revision...\n");
	unsigned char hwRevision = readMemoryByte();
	printf("Revision @ user[16][6] = 0x%x\n", hwRevision);
	printf("Resetting memory bank selection to 0...\n");
	setMemoryBank(0, false, false);

	// check OTP bank valid
	printf("Reading OTP bank valid flag...\n");
	unsigned char otpValid = getOTPBankValid();
	printf("OTP bank is %s\n", otpValid ? "valid!" : "invalid!");

	// get X/Y/Z gyro offsets
	printf("Reading gyro offset values...\n");
	char xgOffset = getXGyroOffset();
	char ygOffset = getYGyroOffset();
	char zgOffset = getZGyroOffset();
	printf("X gyro offset = %d\n", xgOffset);
	printf("Y gyro offset = %d\n", ygOffset);
	printf("Z gyro offset = %d\n", ygOffset);

	readByte(devAddr, MPU6050_RA_USER_CTRL, buffer); // ?

	printf("Enabling interrupt latch, clear on any read, AUX bypass enabled\n");

	writeByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0x32);

	// enable MPU AUX I2C bypass mode
	//DEBUG_PRINTLN(F("Enabling AUX I2C bypass mode..."));
	//setI2CBypassEnabled(true);

	printf("Setting magnetometer mode to power-down...\n");
	//mag -> setMode(0);
	writeByte(0x0E, 0x0A, 0x00);

	printf("Setting magnetometer mode to fuse access...\n");
	//mag -> setMode(0x0F);
	writeByte(0x0E, 0x0A, 0x0F);

	printf("Reading mag magnetometer factory calibration...\n");
	char asax, asay, asaz;
	//mag -> getAdjustment(&asax, &asay, &asaz);
	readBytes(0x0E, 0x10, 3, buffer);
	asax = (int8_t) buffer[0];
	asay = (int8_t) buffer[1];
	asaz = (int8_t) buffer[2];
	printf("Adjustment X/Y/Z = %d/%d/%d\n", asax, asay, asaz);

	printf("Setting magnetometer mode to power-down...\n");
	//mag -> setMode(0);
	writeByte(0x0E, 0x0A, 0x00);

	// load DMP code into memory banks
	printf("Writing DMP code to MPU memory banks (%d bytes)\n",
	MPU6050_DMP_CODE_SIZE);

	if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true)) {
		printf("Success! DMP code written and verified.\n");

		printf("Configuring DMP and related settings...\n");

		// write DMP configuration
		printf(
				"Writing DMP configuration to MPU memory banks (%d bytes in config def)\n",
				MPU6050_DMP_CONFIG_SIZE);

		if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
			printf("Success! DMP configuration written and verified.\n");

			printf("Setting DMP and FIFO_OFLOW interrupts enabled...\n");
			setIntEnabled(0x12);

			printf("Setting sample rate to 200Hz...\n");
			setRate(4); // 1khz / (1 + 4) = 200 Hz

			printf("Setting clock source to Z Gyro...\n");
			setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

			//20 HZ is for mpu9250,  too much noise......
			printf("Setting DLPF bandwidth to 20Hz...\n");
			setDLPFMode(MPU6050_DLPF_BW_20); 

			printf("Setting external frame sync to TEMP_OUT_L[0]...\n");
			setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

			printf("Setting gyro sensitivity to +/- 2000 deg/sec...\n");
			setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
			printf("Setting DMP configuration bytes (function unknown)...\n");
			setDMPConfig1(0x03);
			setDMPConfig2(0x00);

			printf("Clearing OTP Bank flag...\n");
			setOTPBankValid(false);

			printf("Setting X/Y/Z gyro offsets to previous values...\n");
			setXGyroOffsetTC(xgOffset);
			setYGyroOffsetTC(ygOffset);
			setZGyroOffsetTC(zgOffset);

			printf("Setting X/Y/Z gyro user offsets to %d/%d/%d...\n",
					xGyroOffset, yGyroOffset, zGyroOffset);
			setXGyroOffsetUser(xGyroOffset);
			setYGyroOffsetUser(yGyroOffset);
			setZGyroOffsetUser(zGyroOffset);

			printf("Writing final memory update 1/19 (function unknown)...\n");
			unsigned char dmpUpdate[16], j;
			unsigned short pos = 0;
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Writing final memory update 2/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Resetting FIFO...\n");
			resetFIFO();

			printf("Reading FIFO count...\n");
			unsigned char fifoCount = getFIFOCount();

			printf("Current FIFO count=%d\n", fifoCount);
			unsigned char fifoBuffer[128];
			//getFIFOBytes(fifoBuffer, fifoCount);

			printf("Writing final memory update 3/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Writing final memory update 4/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Disabling all standby flags...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS , MPU6050_RA_PWR_MGMT_2, 0x00);

			printf("Setting accelerometer sensitivity to +/- 8g...\n");
			setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

			printf("Setting motion detection threshold to 2...\n");
			setMotionDetectionThreshold(2);

			printf("Setting zero-motion detection threshold to 156...\n");
			setZeroMotionDetectionThreshold(156);

			printf("Setting motion detection duration to 80...\n");
			setMotionDetectionDuration(80);

			printf("Setting zero-motion detection duration to 0...\n");
			setZeroMotionDetectionDuration(0);

			printf("Setting AK8975 to single measurement mode...\n");
			//mag -> setMode(1);
			writeByte(0x0E, 0x0A, 0x01);

			// setup AK8975 (0x0E) as Slave 0 in read mode
			printf("Setting up AK8975 read slave 0...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x8E);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x01);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0xDA);

			// setup AK8975 (0x0E) as Slave 2 in write mode
			printf("Setting up AK8975 write slave 2...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x0E);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x0A);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x81);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x01);

			// setup I2C timing/delay control
			printf("Setting up slave access delay...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x18);
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x05);

			// enable interrupts
			printf("Enabling default interrupt behavior/no bypass...\n");
			writeByte(0x68, MPU6050_RA_INT_PIN_CFG, 0x00);

			// enable I2C master mode and reset DMP/FIFO
			printf("Enabling I2C master mode...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x20);
			printf("Resetting FIFO...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x24);
			printf(
					"Rewriting I2C master mode enabled because...I don't know\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x20);
			printf("Enabling and resetting DMP/FIFO...\n");
			writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0xE8);

			printf("Writing final memory update 5/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 6/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 7/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 8/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 9/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 10/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 11/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Reading final memory update 12/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1]);
#if 0
			printf("Read bytes: ");
			for (j = 0; j < 4; j++) {
				printf("%d",dmpUpdate[3 + j]);
				printf(" ");
			}
			printf("\n");
#endif

			printf("Writing final memory update 13/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 14/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 15/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 16/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);
			printf("Writing final memory update 17/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Waiting for FIRO count >= 46...\n");
			while ((fifoCount = getFIFOCount()) < 46)
				;
			printf("Reading FIFO...\n");
			getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
			printf("Reading interrupt status...");
			getIntStatus();

			printf("Writing final memory update 18/19 (function unknown)..\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Waiting for FIRO count >= 48...\n");
			while ((fifoCount = getFIFOCount()) < 48)
				;
			printf("Reading FIFO...\n");
			getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
			printf("Reading interrupt status...\n");
			getIntStatus();
			printf("Waiting for FIRO count >= 48...\n");
			while ((fifoCount = getFIFOCount()) < 48)
				;
			printf("Reading FIFO...\n");
			getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
			printf("Reading interrupt status...\n");
			getIntStatus();

			printf("Writing final memory update 19/19 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
				dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0],
					dmpUpdate[1], true, false);

			printf("Disabling DMP (you turn it on later)...\n");
			setDMPEnabled(false);

			printf(
					"Setting up internal 48-byte (default) DMP packet buffer...\n");
			dmpPacketSize = 48;
			/*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
			 return 3; // TODO: proper error code for no memory
			 }*/

			printf("Resetting FIFO and clearing INT status one last time...\n");
			resetFIFO();
			getIntStatus();
		} else {
			printf("ERROR! DMP configuration verification failed.\n");
			return 2; // configuration block loading failed
		}
	} else {
		printf("ERROR! DMP code verification failed.\n");
		return 1; // main binary block loading failed
	}
	return 0; // success
}

unsigned char dmpGetMag(short *data, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (packet[28] << 8) | packet[29];
	data[1] = (packet[30] << 8) | packet[31];
	data[2] = (packet[32] << 8) | packet[33];
	return 0;
}

unsigned char dmpGetAccel(short *data, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0)
		packet = dmpPacketBuffer;
	data[0] = (packet[34] << 8) | packet[35];
	data[1] = (packet[38] << 8) | packet[39];
	data[2] = (packet[42] << 8) | packet[43];
	return 0;
}

#else

unsigned char dmpInitialize() {
	// reset device
	printf("\n\nResetting MPU6050 6 AXIS ...\n");
	reset();
	usleep(30000);// wait after reset

	// enable sleep mode and wake cycle
	/*Serial.println(F("Enabling sleep mode..."));
	 setSleepEnabled(true);
	 Serial.println(F("Enabling wake cycle..."));
	 setWakeCycleEnabled(true);*/

	// disable sleep mode
	printf("Disabling sleep mode...\n");
	setSleepEnabled(false);

	// get MPU hardware revision
	printf("Selecting user bank 16...\n");
	setMemoryBank(0x10, true, true);
	printf("Selecting memory byte 6...\n");
	setMemoryStartAddress(0x06);
	printf("Checking hardware revision...\n");
	unsigned char hwRevision __attribute__((__unused__)) = readMemoryByte();
	printf("Revision @ user[16][6] = %x\n",hwRevision);
	printf("Resetting memory bank selection to 0...\n");
	setMemoryBank(0, false, false);

	// check OTP bank valid
	printf("Reading OTP bank valid flag...\n");
	unsigned char otpValid __attribute__((__unused__)) = getOTPBankValid();
	printf("OTP bank is %s\n",otpValid ?"valid!":"invalid!");

	// get X/Y/Z gyro offsets
	printf("Reading gyro offset values...\n");
	char xgOffset = getXGyroOffset();
	char ygOffset = getYGyroOffset();
	char zgOffset = getZGyroOffset();
	printf("X gyro offset = %d\n",xgOffset);
	printf("Z gyro offset = %d\n",ygOffset);
	printf("Z gyro offset = %d\n",zgOffset);

	// setup weird slave stuff (?)
	printf("Setting slave 0 address to 0x7F...\n");
	setSlaveAddress(0, 0x7F);
	printf("Disabling I2C Master mode...\n");
	setI2CMasterModeEnabled(false);
	printf("Setting slave 0 address to %f (self)...\n",MPU6050_DEFAULT_ADDRESS );
	setSlaveAddress(0, MPU6050_DEFAULT_ADDRESS );
	printf("Resetting I2C Master control...\n");
	resetI2CMaster();
	usleep(20000);

	// load DMP code into memory banks
	printf("Writing DMP code to MPU memory banks (%d bytes)\n",MPU6050_DMP_CODE_SIZE);
	if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE,0,0,true)) {
		printf("Success! DMP code written and verified.\n");

		// write DMP configuration
		printf("Writing DMP configuration to MPU memory banks (%d  bytes in config def)\n",MPU6050_DMP_CONFIG_SIZE);
		if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {

			printf("Success! DMP configuration written and verified.\n");

			printf("Setting clock source to Z Gyro...\n");
			setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

			printf("Setting DMP and FIFO_OFLOW interrupts enabled...\n");
			setIntEnabled(0x12);

			printf("Setting sample rate to 200Hz...\n");
			setRate(4); // 1khz / (1 +4 ) = 200 Hz

			printf("Setting external frame sync to TEMP_OUT_L[0]...\n");
			setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

			//printf("Setting DLPF bandwidth to 42Hz...\n");
			//setDLPFMode(MPU6050_DLPF_BW_42);

        		//20 HZ is for mpu9250,  too much noise......
                        printf("Setting DLPF bandwidth to 20Hz...\n");
                        setDLPFMode(MPU6050_DLPF_BW_20);

			printf("Setting gyro sensitivity to +/- 2000 deg/sec...\n");
			setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

//                        printf("Setting gyro sensitivity to +/- 250 deg/sec...\n");
			//                      setFullScaleGyroRange(MPU6050_GYRO_FS_250);

			printf("Setting DMP configuration bytes (function unknown)...\n");
			setDMPConfig1(0x03);
			setDMPConfig2(0x00);

			printf("Clearing OTP Bank flag...\n");
			setOTPBankValid(false);

			printf("Setting X/Y/Z gyro offsets to previous values...\n");
			setXGyroOffset(xgOffset);
			setYGyroOffset(ygOffset);
			setZGyroOffset(zgOffset);

			printf("Setting X/Y/Z gyro user offsets to %d/%d/%d...\n",xGyroOffset,yGyroOffset,zGyroOffset);
			setXGyroOffsetUser(xGyroOffset);
			setYGyroOffsetUser(yGyroOffset);
			setZGyroOffsetUser(zGyroOffset);

			printf("Writing final memory update 1/7 (function unknown)...\n");
			unsigned char dmpUpdate[16], j;
			unsigned short pos = 0;
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

			printf("Writing final memory update 2/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

			printf("Resetting FIFO...\n");
			resetFIFO();

			printf("Reading FIFO count...\n");
			unsigned char fifoCount = getFIFOCount();
			unsigned char fifoBuffer[1024];

			printf("Current FIFO count=%d\n", fifoCount);
			if (fifoCount > 0)
			getFIFOBytes(fifoBuffer, fifoCount);

			printf("Setting motion detection threshold to 2...\n");
			setMotionDetectionThreshold(2);

			printf("Setting zero-motion detection threshold to 156...\n");
			setZeroMotionDetectionThreshold(156);

			printf("Setting motion detection duration to 80...\n");
			setMotionDetectionDuration(80);

			printf("Setting zero-motion detection duration to 0...\n");
			setZeroMotionDetectionDuration(0);

			printf("Resetting FIFO...\n");
			resetFIFO();

			printf("Enabling FIFO...\n");
			setFIFOEnabled(true);

			printf("Enabling DMP...\n");
			setDMPEnabled(true);

			printf("Resetting DMP...\n");
			resetDMP();

			printf("Writing final memory update 3/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

			printf("Writing final memory update 4/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

			printf("Writing final memory update 5/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

			printf("Waiting for FIFO count > 2...\n");
			while ((fifoCount = getFIFOCount()) < 3);

			printf("Current FIFO count=%d\n",fifoCount);
			printf("Reading FIFO data...\n");
			getFIFOBytes(fifoBuffer, fifoCount);

			printf("Reading interrupt status...\n");
			unsigned char mpuIntStatus __attribute__((__unused__)) = getIntStatus();
			printf("Current interrupt status=%d\n",mpuIntStatus);

			printf("Reading final memory update 6/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

			printf("Waiting for FIFO count > 2...\n");
			while ((fifoCount = getFIFOCount()) < 3);
			printf("Current FIFO count=%d\n",fifoCount);

			printf("Reading FIFO data...\n");
			getFIFOBytes(fifoBuffer, fifoCount);

			printf("Reading interrupt status...\n");
			mpuIntStatus = getIntStatus();

			printf("Current interrupt status=%d\n",mpuIntStatus);

			printf("Writing final memory update 7/7 (function unknown)...\n");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);
			printf("!!!!!!!!!DMP is good to go! Finally!!!!!!!!!!\n");

			printf("Disabling DMP (you turn it on later)...\n");
			setDMPEnabled(false);

			printf("Setting up internal 42-byte (default) DMP packet buffer...\n");
			dmpPacketSize = 42;
			//if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
			//	return 3; // TODO: proper error code for no memory
			//}

			printf("Resetting FIFO and clearing INT status one last time...\n");
			resetFIFO();
			getIntStatus();
		} else {
			printf("ERROR! DMP configuration verification failed.\n");
			return 2; // configuration block loading failed
		}

	} else {
		printf("ERROR! DMP code verification failed.\n");
		return 2; // main binary block loading failed
	}
	printf("%s %d: success.\n",__func__,__LINE__);
	return 0; // success
}

unsigned char dmpGetAccel(short *data, const unsigned char* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0) packet = dmpPacketBuffer;
	data[0] = (packet[28] << 8) | packet[29];
	data[1] = (packet[32] << 8) | packet[33];
	data[2] = (packet[36] << 8) | packet[37];
	return 0;
}

#endif

