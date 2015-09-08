#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"
	
#include "soft_iic.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define devAddr						  MPU6050_DEFAULT_ADDRESS

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
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
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

#define MPU6050_SELF_TEST_XA_1_BIT     0x07
#define MPU6050_SELF_TEST_XA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_XA_2_BIT     0x05
#define MPU6050_SELF_TEST_XA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_YA_1_BIT     0x07
#define MPU6050_SELF_TEST_YA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_YA_2_BIT     0x03
#define MPU6050_SELF_TEST_YA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_ZA_1_BIT     0x07
#define MPU6050_SELF_TEST_ZA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_ZA_2_BIT     0x01
#define MPU6050_SELF_TEST_ZA_2_LENGTH  0x02

#define MPU6050_SELF_TEST_XG_1_BIT     0x04
#define MPU6050_SELF_TEST_XG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_YG_1_BIT     0x04
#define MPU6050_SELF_TEST_YG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_ZG_1_BIT     0x04
#define MPU6050_SELF_TEST_ZG_1_LENGTH  0x05

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

// TODO: figure out what these actually do
// UMPL source code is not very obivous
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


uint32_t MPU6050_read_byte(uint8_t reg, uint8_t *dat);
uint32_t MPU6050_read_bytes(uint8_t reg, uint8_t *dat, uint32_t len);
uint32_t MPU6050_write_byte(uint8_t reg, uint8_t *dat);
uint32_t MPU6050_write_bytes(uint8_t reg, uint8_t *dat, uint32_t len);
uint32_t MPU6050_write_bits(uint8_t reg, uint8_t bit_start, 
	uint8_t bit_length, uint8_t dat);
uint32_t MPU6050_write_bit(uint8_t regAddr, uint8_t bitNum, uint8_t data) ;
uint32_t MPU6050_read_bits(uint8_t reg, uint8_t bit_start, 
	uint8_t bit_length, uint8_t *dat);
uint32_t MPU6050_read_bit(uint8_t regAddr, uint8_t bitNum, uint8_t *dat) ;
uint32_t MPU6050_initialize(void);
uint32_t testConnection(void);

// AUX_VDDIO register
uint8_t getAuxVDDIOLevel(void);
void setAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
uint8_t getRate(void);
void setRate(uint8_t rate);

// CONFIG register
uint8_t getExternalFrameSync(void);
void setExternalFrameSync(uint8_t sync);
uint8_t getDLPFMode(void);
void setDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
uint8_t getFullScaleGyroRange(void);
void setFullScaleGyroRange(uint8_t range);

// SELF_TEST registers
uint8_t getAccelXSelfTestFactoryTrim(void);
uint8_t getAccelYSelfTestFactoryTrim(void);
uint8_t getAccelZSelfTestFactoryTrim(void);

uint8_t getGyroXSelfTestFactoryTrim(void);
uint8_t getGyroYSelfTestFactoryTrim(void);
uint8_t getGyroZSelfTestFactoryTrim(void);

// ACCEL_CONFIG register
uint32_t getAccelXSelfTest(void);
void setAccelXSelfTest(uint32_t enabled);
uint32_t getAccelYSelfTest(void);
void setAccelYSelfTest(uint32_t enabled);
uint32_t getAccelZSelfTest(void);
void setAccelZSelfTest(uint32_t enabled);
uint8_t getFullScaleAccelRange(void);
void setFullScaleAccelRange(uint8_t range);
uint8_t getDHPFMode(void);
void setDHPFMode(uint8_t mode);

// FF_THR register
uint8_t getFreefallDetectionThreshold(void);
void setFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
uint8_t getFreefallDetectionDuration(void);
void setFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
uint8_t getMotionDetectionThreshold(void);
void setMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
uint8_t getMotionDetectionDuration(void);
void setMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
uint8_t getZeroMotionDetectionThreshold(void);
void setZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
uint8_t getZeroMotionDetectionDuration(void);
void setZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
uint32_t getTempFIFOEnabled(void);
void setTempFIFOEnabled(uint32_t enabled);
uint32_t getXGyroFIFOEnabled(void);
void setXGyroFIFOEnabled(uint32_t enabled);
uint32_t getYGyroFIFOEnabled(void);
void setYGyroFIFOEnabled(uint32_t enabled);
uint32_t getZGyroFIFOEnabled(void);
void setZGyroFIFOEnabled(uint32_t enabled);
uint32_t getAccelFIFOEnabled(void);
void setAccelFIFOEnabled(uint32_t enabled);
uint32_t getSlave2FIFOEnabled(void);
void setSlave2FIFOEnabled(uint32_t enabled);
uint32_t getSlave1FIFOEnabled(void);
void setSlave1FIFOEnabled(uint32_t enabled);
uint32_t getSlave0FIFOEnabled(void);
void setSlave0FIFOEnabled(uint32_t enabled);

// I2C_MST_CTRL register
uint32_t getMultiMasterEnabled(void);
void setMultiMasterEnabled(uint32_t enabled);
uint32_t getWaitForExternalSensorEnabled(void);
void setWaitForExternalSensorEnabled(uint32_t enabled);
uint32_t getSlave3FIFOEnabled(void);
void setSlave3FIFOEnabled(uint32_t enabled);
uint32_t getSlaveReadWriteTransitionEnabled(void);
void setSlaveReadWriteTransitionEnabled(uint32_t enabled);
uint8_t getMasterClockSpeed(void);
void setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t getSlaveAddress(uint8_t num);
void setSlaveAddress(uint8_t num, uint8_t address);
uint8_t getSlaveRegister(uint8_t num);
void setSlaveRegister(uint8_t num, uint8_t reg);
uint32_t getSlaveEnabled(uint8_t num);
void setSlaveEnabled(uint8_t num, uint32_t enabled);
uint32_t getSlaveWordByteSwap(uint8_t num);
void setSlaveWordByteSwap(uint8_t num, uint32_t enabled);
uint32_t getSlaveWriteMode(uint8_t num);
void setSlaveWriteMode(uint8_t num, uint32_t mode);
uint32_t getSlaveWordGroupOffset(uint8_t num);
void setSlaveWordGroupOffset(uint8_t num, uint32_t enabled);
uint8_t getSlaveDataLength(uint8_t num);
void setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t getSlave4Address(void);
void setSlave4Address(uint8_t address);
uint8_t getSlave4Register(void);
void setSlave4Register(uint8_t reg);
void setSlave4OutputByte(uint8_t data);
uint32_t getSlave4Enabled(void);
void setSlave4Enabled(uint32_t enabled);
uint32_t getSlave4InterruptEnabled(void);
void setSlave4InterruptEnabled(uint32_t enabled);
uint32_t getSlave4WriteMode(void);
void setSlave4WriteMode(uint32_t mode);
uint8_t getSlave4MasterDelay(void);
void setSlave4MasterDelay(uint8_t delay);
uint8_t getSlate4InputByte(void);

// I2C_MST_STATUS register
uint32_t getPassthroughStatus(void);
uint32_t getSlave4IsDone(void);
uint32_t getLostArbitration(void);
uint32_t getSlave4Nack(void);
uint32_t getSlave3Nack(void);
uint32_t getSlave2Nack(void);
uint32_t getSlave1Nack(void);
uint32_t getSlave0Nack(void);

// INT_PIN_CFG register
uint32_t getInterruptMode(void);
void setInterruptMode(uint32_t mode);
uint32_t getInterruptDrive(void);
void setInterruptDrive(uint32_t drive);
uint32_t getInterruptLatch(void);
void setInterruptLatch(uint32_t latch);
uint32_t getInterruptLatchClear(void);
void setInterruptLatchClear(uint32_t clear);
uint32_t getFSyncInterruptLevel(void);
void setFSyncInterruptLevel(uint32_t level);
uint32_t getFSyncInterruptEnabled(void);
void setFSyncInterruptEnabled(uint32_t enabled);
uint32_t getI2CBypassEnabled(void);
void setI2CBypassEnabled(uint32_t enabled);
uint32_t getClockOutputEnabled(void);
void setClockOutputEnabled(uint32_t enabled);

// INT_ENABLE register
uint8_t getIntEnabled(void);
void setIntEnabled(uint8_t enabled);
uint32_t getIntFreefallEnabled(void);
void setIntFreefallEnabled(uint32_t enabled);
uint32_t getIntMotionEnabled(void);
void setIntMotionEnabled(uint32_t enabled);
uint32_t getIntZeroMotionEnabled(void);
void setIntZeroMotionEnabled(uint32_t enabled);
uint32_t getIntFIFOBufferOverflowEnabled(void);
void setIntFIFOBufferOverflowEnabled(uint32_t enabled);
uint32_t getIntI2CMasterEnabled(void);
void setIntI2CMasterEnabled(uint32_t enabled);
uint32_t getIntDataReadyEnabled(void);
void setIntDataReadyEnabled(uint32_t enabled);

// INT_STATUS register
uint8_t getIntStatus(void);
uint32_t getIntFreefallStatus(void);
uint32_t getIntMotionStatus(void);
uint32_t getIntZeroMotionStatus(void);
uint32_t getIntFIFOBufferOverflowStatus(void);
uint32_t getIntI2CMasterStatus(void);
uint32_t getIntDataReadyStatus(void);

// ACCEL_*OUT_* registers
void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t getAccelerationX(void);
int16_t getAccelerationY(void);
int16_t getAccelerationZ(void);

// TEMP_OUT_* registers
int16_t getTemperature(void);

// GYRO_*OUT_* registers
void getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t getRotationX(void);
int16_t getRotationY(void);
int16_t getRotationZ(void);

// EXT_SENS_DATA_* registers
uint8_t getExternalSensorByte(int position);
uint16_t getExternalSensorWord(int position);
uint32_t getExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
uint8_t getMotionStatus(void);
uint32_t getXNegMotionDetected(void);
uint32_t getXPosMotionDetected(void);
uint32_t getYNegMotionDetected(void);
uint32_t getYPosMotionDetected(void);
uint32_t getZNegMotionDetected(void);
uint32_t getZPosMotionDetected(void);
uint32_t getZeroMotionDetected(void);

// I2C_SLV*_DO register
void setSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
uint32_t getExternalShadowDelayEnabled(void);
void setExternalShadowDelayEnabled(uint32_t enabled);
uint32_t getSlaveDelayEnabled(uint8_t num);
void setSlaveDelayEnabled(uint8_t num, uint32_t enabled);

// SIGNAL_PATH_RESET register
void resetGyroscopePath(void);
void resetAccelerometerPath(void);
void resetTemperaturePath(void);

// MOT_DETECT_CTRL register
uint8_t getAccelerometerPowerOnDelay(void);
void setAccelerometerPowerOnDelay(uint8_t delay);
uint8_t getFreefallDetectionCounterDecrement(void);
void setFreefallDetectionCounterDecrement(uint8_t decrement);
uint8_t getMotionDetectionCounterDecrement(void);
void setMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
uint32_t getFIFOEnabled(void);
void setFIFOEnabled(uint32_t enabled);
uint32_t getI2CMasterModeEnabled(void);
void setI2CMasterModeEnabled(uint32_t enabled);
void switchSPIEnabled(uint32_t enabled);
void resetFIFO(void);
void resetI2CMaster(void);
void resetSensors(void);

// PWR_MGMT_1 register
uint32_t reset(void);
uint32_t getSleepEnabled(void);
void setSleepEnabled(uint32_t enabled);
uint32_t getWakeCycleEnabled(void);
void setWakeCycleEnabled(uint32_t enabled);
uint32_t getTempSensorEnabled(void);
void setTempSensorEnabled(uint32_t enabled);
uint8_t getClockSource(void);
void setClockSource(uint8_t source);

// PWR_MGMT_2 register
uint8_t getWakeFrequency(void);
void setWakeFrequency(uint8_t frequency);
uint32_t getStandbyXAccelEnabled(void);
void setStandbyXAccelEnabled(uint32_t enabled);
uint32_t getStandbyYAccelEnabled(void);
void setStandbyYAccelEnabled(uint32_t enabled);
uint32_t getStandbyZAccelEnabled(void);
void setStandbyZAccelEnabled(uint32_t enabled);
uint32_t getStandbyXGyroEnabled(void);
void setStandbyXGyroEnabled(uint32_t enabled);
uint32_t getStandbyYGyroEnabled(void);
void setStandbyYGyroEnabled(uint32_t enabled);
uint32_t getStandbyZGyroEnabled(void);
void setStandbyZGyroEnabled(uint32_t enabled);

// FIFO_COUNT_* registers
uint16_t getFIFOCount(void);

// FIFO_R_W register
uint8_t getFIFOByte(void);
void setFIFOByte(uint8_t data);
void getFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t getDeviceID(void);
void setDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t getOTPBankValid(void);
void setOTPBankValid(uint32_t enabled);
int8_t getXGyroOffsetTC(void);
void setXGyroOffsetTC(int8_t offset);

// YG_OFFS_TC register
int8_t getYGyroOffsetTC(void);
void setYGyroOffsetTC(int8_t offset);

// ZG_OFFS_TC register
int8_t getZGyroOffsetTC(void);
void setZGyroOffsetTC(int8_t offset);

// X_FINE_GAIN register
int8_t getXFineGain(void);
void setXFineGain(uint8_t gain);

// Y_FINE_GAIN register
int8_t getYFineGain(void);
void setYFineGain(uint8_t gain);

// Z_FINE_GAIN register
int8_t getZFineGain(void);
void setZFineGain(uint8_t gain);

// XA_OFFS_* registers
int16_t getXAccelOffset(void);
void setXAccelOffset(int16_t offset);

// YA_OFFS_* register
int16_t getYAccelOffset(void);
void setYAccelOffset(int16_t offset);

// ZA_OFFS_* register
int16_t getZAccelOffset(void);
void setZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
int16_t getXGyroOffset(void);
void setXGyroOffset(int16_t offset);

// YG_OFFS_USR* register
int16_t getYGyroOffset(void);
void setYGyroOffset(int16_t offset);

// ZG_OFFS_USR* register
int16_t getZGyroOffset(void);
void setZGyroOffset(int16_t offset);

// INT_ENABLE register (DMP functions)
uint32_t getIntPLLReadyEnabled(void);
void setIntPLLReadyEnabled(uint32_t enabled);
uint32_t getIntDMPEnabled(void);
void setIntDMPEnabled(uint32_t enabled) ;
uint32_t getDMPInt5Status(void);
uint32_t getDMPInt4Status(void);
uint32_t getDMPInt3Status(void);
uint32_t getDMPInt2Status(void);
uint32_t getDMPInt1Status(void);
uint32_t getDMPInt0Status(void);
uint32_t getIntPLLReadyStatus(void) ;
uint32_t getIntDMPStatus(void) ;
uint32_t getDMPEnabled(void) ;
void setDMPEnabled(uint32_t enabled);
void resetDMP(void) ;
uint32_t setMemoryBank(uint8_t bank, uint32_t prefetchEnabled, uint32_t userBank) ;
uint32_t setMemoryStartAddress(uint8_t address) ;
uint32_t readMemoryByte(void) ;
uint32_t writeMemoryByte(uint8_t data) ;
void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) ;
uint32_t writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint32_t verify, uint32_t useProgMem) ;
uint32_t writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint32_t verify) ;
uint32_t writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint32_t useProgMem) ;
uint32_t writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) ;
uint32_t getDMPConfig1(void);
void setDMPConfig1(uint8_t config) ;
uint32_t getDMPConfig2(void) ;
void setDMPConfig2(uint8_t config) ;

#ifdef __cplusplus
}
#endif

#endif

