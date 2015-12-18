/*
===============================================
BMI160 accelerometer/gyroscope library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.

Based on MPU6050 Arduino library provided by Jeff Rowberg as part of his
excellent I2Cdev device library: https://github.com/jrowberg/i2cdevlib

===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _CURIE_IMU_H_
#define _CURIE_IMU_H_

#include "Arduino.h"

#define CURIE_IMU_SPI_READ_BIT         7

#define CURIE_IMU_RA_CHIP_ID           0x00

#define CURIE_IMU_ACC_PMU_STATUS_BIT   4
#define CURIE_IMU_ACC_PMU_STATUS_LEN   2
#define CURIE_IMU_GYR_PMU_STATUS_BIT   2
#define CURIE_IMU_GYR_PMU_STATUS_LEN   2

#define CURIE_IMU_RA_PMU_STATUS        0x03

#define CURIE_IMU_RA_GYRO_X_L          0x0C
#define CURIE_IMU_RA_GYRO_X_H          0x0D
#define CURIE_IMU_RA_GYRO_Y_L          0x0E
#define CURIE_IMU_RA_GYRO_Y_H          0x0F
#define CURIE_IMU_RA_GYRO_Z_L          0x10
#define CURIE_IMU_RA_GYRO_Z_H          0x11
#define CURIE_IMU_RA_ACCEL_X_L         0x12
#define CURIE_IMU_RA_ACCEL_X_H         0x13
#define CURIE_IMU_RA_ACCEL_Y_L         0x14
#define CURIE_IMU_RA_ACCEL_Y_H         0x15
#define CURIE_IMU_RA_ACCEL_Z_L         0x16
#define CURIE_IMU_RA_ACCEL_Z_H         0x17

#define CURIE_IMU_STATUS_FOC_RDY       3
#define CURIE_IMU_STATUS_NVM_RDY       4
#define CURIE_IMU_STATUS_DRDY_GYR      6
#define CURIE_IMU_STATUS_DRDY_ACC      7

#define CURIE_IMU_RA_STATUS            0x1B

#define CURIE_IMU_STEP_INT_BIT         0
#define CURIE_IMU_ANYMOTION_INT_BIT    2
#define CURIE_IMU_D_TAP_INT_BIT        4
#define CURIE_IMU_S_TAP_INT_BIT        5
#define CURIE_IMU_NOMOTION_INT_BIT     7
#define CURIE_IMU_FFULL_INT_BIT        5
#define CURIE_IMU_DRDY_INT_BIT         4
#define CURIE_IMU_LOW_G_INT_BIT        3
#define CURIE_IMU_HIGH_G_INT_BIT       2

#define CURIE_IMU_TAP_SIGN_BIT         7
#define CURIE_IMU_TAP_1ST_Z_BIT        6
#define CURIE_IMU_TAP_1ST_Y_BIT        5
#define CURIE_IMU_TAP_1ST_X_BIT        4

#define CURIE_IMU_ANYMOTION_SIGN_BIT   3
#define CURIE_IMU_ANYMOTION_1ST_Z_BIT  2
#define CURIE_IMU_ANYMOTION_1ST_Y_BIT  1
#define CURIE_IMU_ANYMOTION_1ST_X_BIT  0

#define CURIE_IMU_HIGH_G_SIGN_BIT      3
#define CURIE_IMU_HIGH_G_1ST_Z_BIT     2
#define CURIE_IMU_HIGH_G_1ST_Y_BIT     1
#define CURIE_IMU_HIGH_G_1ST_X_BIT     0

#define CURIE_IMU_RA_INT_STATUS_0      0x1C
#define CURIE_IMU_RA_INT_STATUS_1      0x1D
#define CURIE_IMU_RA_INT_STATUS_2      0x1E
#define CURIE_IMU_RA_INT_STATUS_3      0x1F

#define CURIE_IMU_RA_TEMP_L            0x20
#define CURIE_IMU_RA_TEMP_H            0x21

#define CURIE_IMU_RA_FIFO_LENGTH_0     0x22
#define CURIE_IMU_RA_FIFO_LENGTH_1     0x23

#define CURIE_IMU_FIFO_DATA_INVALID    0x80
#define CURIE_IMU_RA_FIFO_DATA         0x24

#define CURIE_IMU_ACCEL_RATE_SEL_BIT    0
#define CURIE_IMU_ACCEL_RATE_SEL_LEN    4

#define CURIE_IMU_RA_ACCEL_CONF        0X40
#define CURIE_IMU_RA_ACCEL_RANGE       0X41

#define CURIE_IMU_GYRO_RATE_SEL_BIT    0
#define CURIE_IMU_GYRO_RATE_SEL_LEN    4

#define CURIE_IMU_RA_GYRO_CONF         0X42
#define CURIE_IMU_RA_GYRO_RANGE        0X43

#define CURIE_IMU_FIFO_HEADER_EN_BIT   4
#define CURIE_IMU_FIFO_ACC_EN_BIT      6
#define CURIE_IMU_FIFO_GYR_EN_BIT      7

#define CURIE_IMU_RA_FIFO_CONFIG_0     0x46
#define CURIE_IMU_RA_FIFO_CONFIG_1     0x47

#define CURIE_IMU_ANYMOTION_EN_BIT     0
#define CURIE_IMU_ANYMOTION_EN_LEN     3
#define CURIE_IMU_D_TAP_EN_BIT         4
#define CURIE_IMU_S_TAP_EN_BIT         5
#define CURIE_IMU_NOMOTION_EN_BIT      0
#define CURIE_IMU_NOMOTION_EN_LEN      3
#define CURIE_IMU_LOW_G_EN_BIT         3
#define CURIE_IMU_LOW_G_EN_LEN         1
#define CURIE_IMU_HIGH_G_EN_BIT        0
#define CURIE_IMU_HIGH_G_EN_LEN        3

#define CURIE_IMU_STEP_EN_BIT          3
#define CURIE_IMU_DRDY_EN_BIT          4
#define CURIE_IMU_FFULL_EN_BIT         5

#define CURIE_IMU_RA_INT_EN_0          0x50
#define CURIE_IMU_RA_INT_EN_1          0x51
#define CURIE_IMU_RA_INT_EN_2          0x52

#define CURIE_IMU_INT1_EDGE_CTRL       0
#define CURIE_IMU_INT1_LVL             1
#define CURIE_IMU_INT1_OD              2
#define CURIE_IMU_INT1_OUTPUT_EN       3

#define CURIE_IMU_RA_INT_OUT_CTRL      0x53

#define CURIE_IMU_LATCH_MODE_BIT       0
#define CURIE_IMU_LATCH_MODE_LEN       4

#define CURIE_IMU_RA_INT_LATCH         0x54
#define CURIE_IMU_RA_INT_MAP_0         0x55
#define CURIE_IMU_RA_INT_MAP_1         0x56
#define CURIE_IMU_RA_INT_MAP_2         0x57

#define CURIE_IMU_ANYMOTION_DUR_BIT    0
#define CURIE_IMU_ANYMOTION_DUR_LEN    2
#define CURIE_IMU_NOMOTION_DUR_BIT     2
#define CURIE_IMU_NOMOTION_DUR_LEN     6

#define CURIE_IMU_NOMOTION_SEL_BIT     0
#define CURIE_IMU_NOMOTION_SEL_LEN     1

#define CURIE_IMU_RA_INT_LOWHIGH_0     0x5A
#define CURIE_IMU_RA_INT_LOWHIGH_1     0x5B
#define CURIE_IMU_RA_INT_LOWHIGH_2     0x5C
#define CURIE_IMU_RA_INT_LOWHIGH_3     0x5D
#define CURIE_IMU_RA_INT_LOWHIGH_4     0x5E

#define CURIE_IMU_RA_INT_MOTION_0      0x5F
#define CURIE_IMU_RA_INT_MOTION_1      0x60
#define CURIE_IMU_RA_INT_MOTION_2      0x61
#define CURIE_IMU_RA_INT_MOTION_3      0x62

#define CURIE_IMU_TAP_DUR_BIT          0
#define CURIE_IMU_TAP_DUR_LEN          3
#define CURIE_IMU_TAP_SHOCK_BIT        6
#define CURIE_IMU_TAP_QUIET_BIT        7
#define CURIE_IMU_TAP_THRESH_BIT       0
#define CURIE_IMU_TAP_THRESH_LEN       5

#define CURIE_IMU_RA_INT_TAP_0         0x63
#define CURIE_IMU_RA_INT_TAP_1         0x64

#define CURIE_IMU_FOC_ACC_Z_BIT        0
#define CURIE_IMU_FOC_ACC_Z_LEN        2
#define CURIE_IMU_FOC_ACC_Y_BIT        2
#define CURIE_IMU_FOC_ACC_Y_LEN        2
#define CURIE_IMU_FOC_ACC_X_BIT        4
#define CURIE_IMU_FOC_ACC_X_LEN        2
#define CURIE_IMU_FOC_GYR_EN           6

#define CURIE_IMU_RA_FOC_CONF          0x69

#define CURIE_IMU_GYR_OFFSET_MSB_LEN   2
#define CURIE_IMU_GYR_OFFSET_X_MSB_BIT 0
#define CURIE_IMU_GYR_OFFSET_X_MSB_LEN 2
#define CURIE_IMU_GYR_OFFSET_Y_MSB_BIT 2
#define CURIE_IMU_GYR_OFFSET_Y_MSB_LEN 2
#define CURIE_IMU_GYR_OFFSET_Z_MSB_BIT 4
#define CURIE_IMU_GYR_OFFSET_Z_MSB_LEN 2
#define CURIE_IMU_ACC_OFFSET_EN        6
#define CURIE_IMU_GYR_OFFSET_EN        7

#define CURIE_IMU_RA_OFFSET_0          0x71
#define CURIE_IMU_RA_OFFSET_1          0x72
#define CURIE_IMU_RA_OFFSET_2          0x73
#define CURIE_IMU_RA_OFFSET_3          0x74

#define CURIE_IMU_RA_OFFSET_4          0x75
#define CURIE_IMU_RA_OFFSET_5          0x76
#define CURIE_IMU_RA_OFFSET_6          0x77

#define CURIE_IMU_RA_STEP_CNT_L        0x78
#define CURIE_IMU_RA_STEP_CNT_H        0x79

#define CURIE_IMU_STEP_BUF_MIN_BIT     0
#define CURIE_IMU_STEP_BUF_MIN_LEN     3
#define CURIE_IMU_STEP_CNT_EN_BIT      3

#define CURIE_IMU_STEP_TIME_MIN_BIT    0
#define CURIE_IMU_STEP_TIME_MIN_LEN    3
#define CURIE_IMU_STEP_THRESH_MIN_BIT  3
#define CURIE_IMU_STEP_THRESH_MIN_LEN  2
#define CURIE_IMU_STEP_ALPHA_BIT       5
#define CURIE_IMU_STEP_ALPHA_LEN       3

#define CURIE_IMU_RA_STEP_CONF_0       0x7A
#define CURIE_IMU_RA_STEP_CONF_1       0x7B

#define CURIE_IMU_RA_STEP_CONF_0_NOR   0x15
#define CURIE_IMU_RA_STEP_CONF_0_SEN   0x2D
#define CURIE_IMU_RA_STEP_CONF_0_ROB   0x1D
#define CURIE_IMU_RA_STEP_CONF_1_NOR   0x03
#define CURIE_IMU_RA_STEP_CONF_1_SEN   0x00
#define CURIE_IMU_RA_STEP_CONF_1_ROB   0x07

#define CURIE_IMU_GYRO_RANGE_SEL_BIT   0
#define CURIE_IMU_GYRO_RANGE_SEL_LEN   3

#define CURIE_IMU_GYRO_RATE_SEL_BIT    0
#define CURIE_IMU_GYRO_RATE_SEL_LEN    4

#define CURIE_IMU_GYRO_DLPF_SEL_BIT    4
#define CURIE_IMU_GYRO_DLPF_SEL_LEN    2

#define CURIE_IMU_ACCEL_DLPF_SEL_BIT   4
#define CURIE_IMU_ACCEL_DLPF_SEL_LEN   3

#define CURIE_IMU_ACCEL_RANGE_SEL_BIT  0
#define CURIE_IMU_ACCEL_RANGE_SEL_LEN  4

#define CURIE_IMU_CMD_START_FOC        0x03
#define CURIE_IMU_CMD_ACC_MODE_NORMAL  0x11
#define CURIE_IMU_CMD_GYR_MODE_NORMAL  0x15
#define CURIE_IMU_CMD_FIFO_FLUSH       0xB0
#define CURIE_IMU_CMD_INT_RESET        0xB1
#define CURIE_IMU_CMD_STEP_CNT_CLR     0xB2
#define CURIE_IMU_CMD_SOFT_RESET       0xB6

#define CURIE_IMU_RA_CMD               0x7E

/**
 * Features for getThreshold(), getDuration() functions,
 */
typedef enum {
    CURIE_IMU_FREEFALL = 0,
    CURIE_IMU_SHOCK,
    CURIE_IMU_MOTION,
    CURIE_IMU_ZERO_MOTION,
    CURIE_IMU_STEP,
    CURIE_IMU_TAP,
    CURIE_IMU_TAP_SHOCK,
    CURIE_IMU_TAP_QUIET,
    CURIE_IMU_DOUBLE_TAP,
    CURIE_IMU_FIFO_FULL,
    CURIE_IMU_DATA_READY,
} BMI160Feature;

/**
 * axis options
 *@see autoCalibrateAccelerometerOffset()
 *@see get/setGyroOffset()
 *@see get/setAccelerometerOffset()
 *@see readAcceleration()
 *@see readRotation()
 *@see shockDetected()
 *@see tapDetected()
 *@see motionDetected()
 */
typedef enum{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
}BMI160Axis;

/**
 *direction options
 *@see shockDetected()
 *@see tapDetected()
 *@see motionDetected()
 */
typedef enum{
    POSITIVE,
    NEGATIVE,
} BMI160Direction;

/**
 * Interrupt Latch Mode options
 * @see setInterruptLatch()
 */
typedef enum {
    CURIE_IMU_LATCH_MODE_NONE = 0, /**< Non-latched */
    CURIE_IMU_LATCH_MODE_312_5_US, /**< Temporary, 312.50 microseconds */
    CURIE_IMU_LATCH_MODE_625_US,   /**< Temporary, 625.00 microseconds */
    CURIE_IMU_LATCH_MODE_1_25_MS,  /**< Temporary,   1.25 milliseconds */
    CURIE_IMU_LATCH_MODE_2_5_MS,   /**< Temporary,   2.50 milliseconds */
    CURIE_IMU_LATCH_MODE_5_MS,     /**< Temporary,   5.00 milliseconds */
    CURIE_IMU_LATCH_MODE_10_MS,    /**< Temporary,  10.00 milliseconds */
    CURIE_IMU_LATCH_MODE_20_MS,    /**< Temporary,  20.00 milliseconds */
    CURIE_IMU_LATCH_MODE_40_MS,    /**< Temporary,  40.00 milliseconds */
    CURIE_IMU_LATCH_MODE_80_MS,    /**< Temporary,  80.00 milliseconds */
    CURIE_IMU_LATCH_MODE_160_MS,   /**< Temporary, 160.00 milliseconds */
    CURIE_IMU_LATCH_MODE_320_MS,   /**< Temporary, 320.00 milliseconds */
    CURIE_IMU_LATCH_MODE_640_MS,   /**< Temporary, 640.00 milliseconds */
    CURIE_IMU_LATCH_MODE_1_28_S,   /**< Temporary,   1.28 seconds      */
    CURIE_IMU_LATCH_MODE_2_56_S,   /**< Temporary,   2.56 seconds      */
    CURIE_IMU_LATCH_MODE_LATCH,    /**< Latched, @see resetInterrupt() */
} BMI160InterruptLatchMode;

/**
 * Digital Low-Pass Filter Mode options
 * @see setGyroFilterMode()
 * @see setAccelerometerFilterMode()
 */
typedef enum {
    CURIE_IMU_DLPF_MODE_NORM = 0x2,
    CURIE_IMU_DLPF_MODE_OSR2 = 0x1,
    CURIE_IMU_DLPF_MODE_OSR4 = 0x0,
} BMI160FilterMode;

/**
 * Accelerometer Sensitivity Range options
 * @see setAccelerometerRange()
 */
typedef enum {
    CURIE_IMU_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
    CURIE_IMU_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
    CURIE_IMU_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
    CURIE_IMU_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelerometerRange;

/**
 * Gyroscope Sensitivity Range options
 * @see setGyroRange()
 */
typedef enum {
    CURIE_IMU_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
    CURIE_IMU_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
    CURIE_IMU_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
    CURIE_IMU_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
    CURIE_IMU_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
} BMI160GyroRange;

/**
 * Accelerometer Output Data Rate options
 * @see setAccelRate()
 */
typedef enum {
    CURIE_IMU_ACCEL_RATE_25_2HZ = 5,  /**<   25/2  Hz */
    CURIE_IMU_ACCEL_RATE_25HZ,        /**<   25    Hz */
    CURIE_IMU_ACCEL_RATE_50HZ,        /**<   50    Hz */
    CURIE_IMU_ACCEL_RATE_100HZ,       /**<  100    Hz */
    CURIE_IMU_ACCEL_RATE_200HZ,       /**<  200    Hz */
    CURIE_IMU_ACCEL_RATE_400HZ,       /**<  400    Hz */
    CURIE_IMU_ACCEL_RATE_800HZ,       /**<  800    Hz */
    CURIE_IMU_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
} BMI160AccelerometerRate;

/**
 * Gyroscope Output Data Rate options
 * @see setGyroRate()
 */
typedef enum {
    CURIE_IMU_GYRO_RATE_25HZ = 6,     /**<   25    Hz */
    CURIE_IMU_GYRO_RATE_50HZ,         /**<   50    Hz */
    CURIE_IMU_GYRO_RATE_100HZ,        /**<  100    Hz */
    CURIE_IMU_GYRO_RATE_200HZ,        /**<  200    Hz */
    CURIE_IMU_GYRO_RATE_400HZ,        /**<  400    Hz */
    CURIE_IMU_GYRO_RATE_800HZ,        /**<  800    Hz */
    CURIE_IMU_GYRO_RATE_1600HZ,       /**< 1600    Hz */
    CURIE_IMU_GYRO_RATE_3200HZ,       /**< 3200    Hz */
} BMI160GyroRate;

/**
 * Step Detection Mode options
 * @see setStepDetectionMode()
 */
typedef enum {
    CURIE_IMU_STEP_MODE_NORMAL = 0,
    CURIE_IMU_STEP_MODE_SENSITIVE,
    CURIE_IMU_STEP_MODE_ROBUST,
    CURIE_IMU_STEP_MODE_UNKNOWN,
} BMI160StepMode;

/**
 * Tap Detection Shock Duration options
 * @see setTapShockDuration()
 */
typedef enum {
    CURIE_IMU_TAP_SHOCK_DURATION_50MS = 0,
    CURIE_IMU_TAP_SHOCK_DURATION_75MS,
} BMI160TapShockDuration;

/**
 * Tap Detection Quiet Duration options
 * @see setTapQuietDuration()
 */
typedef enum {
    CURIE_IMU_TAP_QUIET_DURATION_30MS = 0,
    CURIE_IMU_TAP_QUIET_DURATION_20MS,
} BMI160TapQuietDuration;

/**
 * Double-Tap Detection Duration options
 * @see setDoubleTapDetectionDuration()
 */
typedef enum {
    CURIE_IMU_DOUBLE_TAP_DURATION_50MS = 0,
    CURIE_IMU_DOUBLE_TAP_DURATION_100MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_150MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_200MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_250MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_375MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_500MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_700MS,
} BMI160DoubleTapDuration;

/**
 * Zero-Motion Detection Duration options
 * @see setZeroMotionDetectionDuration()
 */
typedef enum {
    CURIE_IMU_ZERO_MOTION_DURATION_1_28S   = 0x00, /**<   1.28 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_2_56S,          /**<   2.56 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_3_84S,          /**<   3.84 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_5_12S,          /**<   5.12 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_6_40S,          /**<   6.40 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_7_68S,          /**<   7.68 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_8_96S,          /**<   8.96 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_10_24S,         /**<  10.24 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_11_52S,         /**<  11.52 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_12_80S,         /**<  12.80 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_14_08S,         /**<  14.08 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_15_36S,         /**<  15.36 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_16_64S,         /**<  16.64 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_17_92S,         /**<  17.92 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_19_20S,         /**<  19.20 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_20_48S,         /**<  20.48 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_25_60S  = 0x10, /**<  25.60 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_30_72S,         /**<  30.72 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_35_84S,         /**<  35.84 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_40_96S,         /**<  40.96 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_46_08S,         /**<  46.08 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_51_20S,         /**<  51.20 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_56_32S,         /**<  56.32 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_61_44S,         /**<  61.44 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_66_56S,         /**<  66.56 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_71_68S,         /**<  71.68 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_76_80S,         /**<  76.80 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_81_92S,         /**<  81.92 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_87_04S,         /**<  87.04 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_92_16S,         /**<  92.16 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_97_28S,         /**<  97.28 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_102_40S,        /**< 102.40 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_112_64S = 0x20, /**< 112.64 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_122_88S,        /**< 122.88 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_133_12S,        /**< 133.12 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_143_36S,        /**< 143.36 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_153_60S,        /**< 153.60 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_163_84S,        /**< 163.84 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_174_08S,        /**< 174.08 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_184_32S,        /**< 184.32 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_194_56S,        /**< 194.56 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_204_80S,        /**< 204.80 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_215_04S,        /**< 215.04 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_225_28S,        /**< 225.28 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_235_52S,        /**< 235.52 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_245_76S,        /**< 245.76 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_256_00S,        /**< 256.00 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_266_24S,        /**< 266.24 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_276_48S,        /**< 276.48 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_286_72S,        /**< 286.72 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_296_96S,        /**< 296.96 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_307_20S,        /**< 307.20 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_317_44S,        /**< 317.44 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_327_68S,        /**< 327.68 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_337_92S,        /**< 337.92 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_348_16S,        /**< 348.16 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_358_40S,        /**< 358.40 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_368_64S,        /**< 368.64 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_378_88S,        /**< 378.88 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_389_12S,        /**< 389.12 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_399_36S,        /**< 399.36 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_409_60S,        /**< 409.60 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_419_84S,        /**< 419.84 seconds */
    CURIE_IMU_ZERO_MOTION_DURATION_430_08S,        /**< 430.08 seconds */
} BMI160ZeroMotionDuration;

class BMI160Class {
    public:
        boolean begin();

        int getGyroRate();
        void setGyroRate(int rate);
        int getAccelerometerRate();
        void setAccelerometerRate(int rate);

        int getGyroFilterMode();
        void setGyroFilterMode(int bandwidth);
        int getAccelerometerFilterMode();
        void setAccelerometerFilterMode(int bandwidth);

        int getGyroRange();
        void setGyroRange(int range);
        int getAccelerometerRange();
        void setAccelerometerRange(int range);

        boolean getGyroFIFOEnabled();
        void setGyroFIFOEnabled(boolean enabled);
        boolean getAccelerometerFIFOEnabled();
        void setAccelerometerFIFOEnabled(boolean enabled);

        void autoCalibrateGyroOffset();
        void autoCalibrateAccelerometerOffset(int axis, int target);

        void enableGyroOffset(boolean state);
        void enableAccelerometerOffset(boolean state);
        boolean gyroOffsetEnabled();
        boolean accelerometerOffsetEnabled();

        int getGyroOffset(int axis);
        int getAccelerometerOffset(int axis);

        void setGyroOffset(int axis, int offset);
        void setAccelerometerOffset(int axis, int offset);

        int getDetectionThreshold(BMI160Feature feature);
        void setDetectionThreshold(BMI160Feature feature, int threshold);

        int getDetectionDuration(BMI160Feature feature);
        void setDetectionDuration(BMI160Feature feature, int value);//value (bool) duration or samples

        void enableInterrupt(BMI160Feature feature, boolean enabled);
        boolean interruptEnabled(BMI160Feature feature);

        int getInterruptBits(int reg); // get raw interrrupt status registers
        int getInterruptStatus(BMI160Feature feature);

        BMI160StepMode getStepDetectionMode();
        void setStepDetectionMode(BMI160StepMode mode);
        long getStepCount();
        boolean getStepCountEnabled();
        void setStepCountEnabled(bool enabled);
        void resetStepCount();

        void readMotionSensor(short& ax, short& ay, short& az, short& gx, short& gy, short& gz);
        void readAcceleration(short& x, short& y, short& z);
        void readRotation(short& x, short& y, short& z);

        int readAccelerometer(int axis);
        int readGyro(int axis);
        int readTemperature();

        boolean shockDetected(int axis, int direction);
        boolean motionDetected(int axis, int direction);
        boolean tapDetected(int axis, int direction);

        boolean getFIFOHeaderModeEnabled();
        void setFIFOHeaderModeEnabled(bool enabled);
        void resetFIFO();
        long getFIFOCount();
        void getFIFOBytes(uint8_t *buffer, uint16_t length);

        int getDeviceID();
        int getRegister(int reg);
        void setRegister(int reg, int data);

        boolean getIntEnabled();
        void setIntEnabled(bool enabled);
        boolean getInterruptMode();
        void setInterruptMode(bool mode);
        boolean getInterruptDrive();
        void setInterruptDrive(bool drive);
        int getInterruptLatch();
        void setInterruptLatch(uint8_t latch);
        void resetInterrupt();

    protected:
        virtual int serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt);

    private:
        uint8_t reg_read (uint8_t reg);
        void reg_write(uint8_t reg, uint8_t data);
        void reg_write_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
        uint8_t reg_read_bits(uint8_t reg, unsigned pos, unsigned len);
};

#endif /* _CURIE_IMU_H_ */
