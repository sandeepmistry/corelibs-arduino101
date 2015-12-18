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
#include "BMI160.h"

#define CURIE_IMU_CHIP_ID 0xD1

#define CURIE_IMU_ACCEL_POWERUP_DELAY_MS 10
#define CURIE_IMU_GYRO_POWERUP_DELAY_MS 100

/* Test the sign bit and set remaining MSBs if sign bit is set */
#define CURIE_IMU_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)

/******************************************************************************/

uint8_t BMI160Class::reg_read (uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = reg;
    serial_buffer_transfer(buffer, 1, 1);
    return buffer[0];
}

void BMI160Class::reg_write(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = data;
    serial_buffer_transfer(buffer, 2, 0);
}

void BMI160Class::reg_write_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
    uint8_t b = reg_read(reg);
    uint8_t mask = ((1 << len) - 1) << pos;
    data <<= pos; // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    reg_write(reg, b);
}

uint8_t BMI160Class::reg_read_bits(uint8_t reg, unsigned pos, unsigned len)
{
    uint16_t b = reg_read(reg);
    uint16_t mask = (1 << len) - 1;
    b >>= pos;
    b &= mask;
    return b;
}

/******************************************************************************/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to default range settings, namely +/- 2g and +/- 250 degrees/sec.
 */
boolean BMI160Class::begin()
{
    /* Issue a soft-reset to bring the device into a clean state */
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_SOFT_RESET);
    delay(1);

    /* Issue a dummy-read to force the device into SPI comms mode */
    reg_read(0x7F);
    delay(1);

    /* Power up the accelerometer */
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_ACC_MODE_NORMAL);
    delay(1);
    /* Wait for power-up to complete */
    while (0x1 != reg_read_bits(CURIE_IMU_RA_PMU_STATUS,
                                CURIE_IMU_ACC_PMU_STATUS_BIT,
                                CURIE_IMU_ACC_PMU_STATUS_LEN))
        delay(1);

    /* Power up the gyroscope */
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_GYR_MODE_NORMAL);
    delay(1);
    /* Wait for power-up to complete */
    while (0x1 != reg_read_bits(CURIE_IMU_RA_PMU_STATUS,
                                CURIE_IMU_GYR_PMU_STATUS_BIT,
                                CURIE_IMU_GYR_PMU_STATUS_LEN))
        delay(1);

    setGyroRange(CURIE_IMU_GYRO_RANGE_250);
    setAccelerometerRange(CURIE_IMU_ACCEL_RANGE_2G);

    /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
    reg_write(CURIE_IMU_RA_INT_MAP_0, 0xFF);
    reg_write(CURIE_IMU_RA_INT_MAP_1, 0xF0);
    reg_write(CURIE_IMU_RA_INT_MAP_2, 0x00);

    /** Verify the SPI connection.
    * MakgetGyroRatee sure the device is connected and responds as expected.
    * @return True if connection is valid, false otherwise
    */
    return (CURIE_IMU_CHIP_ID == getDeviceID());
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b11010001, 0xD1).
 * @return Device ID (should be 0xD1)
 * @see CURIE_IMU_RA_CHIP_ID
 */
int BMI160Class::getDeviceID() {
    return reg_read(CURIE_IMU_RA_CHIP_ID);
}

/** Get gyroscope output data rate.
 * The gyr_odr parameter allows setting the output data rate of the gyroscope
 * as described in the table below.
 *
 * <pre>
 *  6 =   25Hz
 *  7 =   50Hz
 *  8 =  100Hz
 *  9 =  200Hz
 * 10 =  400Hz
 * 11 =  800Hz
 * 12 = 1600Hz
 * 13 = 3200Hz
 * </pre>
 *
 * @return Current sample rate
 * @see CURIE_IMU_RA_GYRO_CONF
 * @see BMI160GyroRate
 */
int BMI160Class::getGyroRate(){
    return reg_read_bits(CURIE_IMU_RA_GYRO_CONF,
                         CURIE_IMU_GYRO_RATE_SEL_BIT,
                         CURIE_IMU_GYRO_RATE_SEL_LEN);
}

/** Set gyroscope output data rate.
 * @param rate New output data rate
 * @see getGyroRate()
 * @see CURIE_IMU_GYRO_RATE_25HZ
 * @see CURIE_IMU_RA_GYRO_CONF
 */
void BMI160Class::setGyroRate(int rate) {
    reg_write_bits(CURIE_IMU_RA_GYRO_CONF, rate,
                   CURIE_IMU_GYRO_RATE_SEL_BIT,
                   CURIE_IMU_GYRO_RATE_SEL_LEN);
}

/** Get accelerometer output data rate.
 * The acc_odr parameter allows setting the output data rate of the accelerometer
 * as described in the table below.
 *
 * <pre>
 *  5 =  25/2Hz
 *  6 =    25Hz
 *  7 =    50Hz
 *  8 =   100Hz
 *  9 =   200Hz
 * 10 =   400Hz
 * 11 =   800Hz
 * 12 =  1600Hz
 * 13 =  3200Hz
 * </pre>
 *
 * @return Current sample rate
 * @see CURIE_IMU_RA_ACCEL_CONF
 * @see BMI160AccelRate
 */
int BMI160Class::getAccelerometerRate() {
    return reg_read_bits(CURIE_IMU_RA_ACCEL_CONF,
                         CURIE_IMU_ACCEL_RATE_SEL_BIT,
                         CURIE_IMU_ACCEL_RATE_SEL_LEN);
}

/** Set accelerometer output data rate.
 * @param rate New output data rate
 * @see getAccelRate()
 * @see CURIE_IMU_RA_ACCEL_CONF
 */
void BMI160Class::setAccelerometerRate(int rate) {
    reg_write_bits(CURIE_IMU_RA_ACCEL_CONF, rate,
                   CURIE_IMU_ACCEL_RATE_SEL_BIT,
                   CURIE_IMU_ACCEL_RATE_SEL_LEN);
}

/** Get gyroscope digital low-pass filter mode.
 * The gyro_bwp parameter sets the gyroscope digital low pass filter configuration.
 *
 * When the filter mode is set to Normal (@see CURIE_IMU_DLPF_MODE_NORM), the filter
 * bandwidth for each respective gyroscope output data rates is shown in the table below:
 *
 * <pre>
 * ODR     | 3dB cut-off
 * --------+------------
 *    25Hz | 10.7Hz
 *    50Hz | 20.8Hz
 *   100Hz | 39.9Hz
 *   200Hz | 74.6Hz
 *   400Hz | 136.6Hz
 *   800Hz | 254.6Hz
 *  1600Hz | 523.9Hz
 *  3200Hz | 890Hz
 * </pre>
 *
 * When the filter mode is set to OSR2 (@see CURIE_IMU_DLPF_MODE_OSR2), the filter
 * bandwidths above are approximately halved.
 *
 * When the filter mode is set to OSR4 (@see CURIE_IMU_DLPF_MODE_OSR4), the filter
 * bandwidths above are approximately 4 times smaller.
 *
 * @return DLFP configuration
 * @see CURIE_IMU_RA_GYRO_CONF
 * @see BMI160DLPFMode
 */
int BMI160Class::getGyroFilterMode() {
    return reg_read_bits(CURIE_IMU_RA_GYRO_CONF,
                         CURIE_IMU_GYRO_DLPF_SEL_BIT,
                         CURIE_IMU_GYRO_DLPF_SEL_LEN);
}

/** Set gyroscope digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getGyroDLPFMode()
 */
void BMI160Class::setGyroFilterMode(int bandwidth) {
    return reg_write_bits(CURIE_IMU_RA_GYRO_CONF, bandwidth,
                          CURIE_IMU_GYRO_DLPF_SEL_BIT,
                          CURIE_IMU_GYRO_DLPF_SEL_LEN);
}

/** Get accelerometer digital low-pass filter mode.
 * The acc_bwp parameter sets the accelerometer digital low pass filter configuration.
 *
 * When the filter mode is set to Normal (@see CURIE_IMU_DLPF_MODE_NORM), the filter
 * bandwidth for each respective accelerometer output data rates is shown in the table below:
 *
 * <pre>
 * ODR     | 3dB cut-off
 * --------+--------------
 *  12.5Hz |  5.06Hz
 *    25Hz | 10.12Hz
 *    50Hz | 20.25Hz
 *   100Hz | 40.5Hz
 *   200Hz | 80Hz
 *   400Hz | 162Hz (155Hz for Z axis)
 *   800Hz | 324Hz (262Hz for Z axis)
 *  1600Hz | 684Hz (353Hz for Z axis)
 * </pre>
 *
 * When the filter mode is set to OSR2 (@see CURIE_IMU_DLPF_MODE_OSR2), the filter
 * bandwidths above are approximately halved.
 *
 * When the filter mode is set to OSR4 (@see CURIE_IMU_DLPF_MODE_OSR4), the filter
 * bandwidths above are approximately 4 times smaller.
 *
 * @return DLFP configuration
 * @see CURIE_IMU_RA_GYRO_CONF
 * @see BMI160DLPFMode
 */
int BMI160Class::getAccelerometerFilterMode() {
    return reg_read_bits(CURIE_IMU_RA_ACCEL_CONF,
                         CURIE_IMU_ACCEL_DLPF_SEL_BIT,
                         CURIE_IMU_ACCEL_DLPF_SEL_LEN);
}

/** Set accelerometer digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getAccelDLPFMode()
 */
void BMI160Class::setAccelerometerFilterMode(int bandwidth) {
    return reg_write_bits(CURIE_IMU_RA_ACCEL_CONF, bandwidth,
                          CURIE_IMU_ACCEL_DLPF_SEL_BIT,
                          CURIE_IMU_ACCEL_DLPF_SEL_LEN);
}

/** Get full-scale gyroscope range.
 * The gyr_range parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 4 = +/-  125 degrees/sec
 * 3 = +/-  250 degrees/sec
 * 2 = +/-  500 degrees/sec
 * 1 = +/- 1000 degrees/sec
 * 0 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see CURIE_IMU_RA_GYRO_RANGE
 * @see BMI160GyroRange
 */
int BMI160Class::getGyroRange() {
    return reg_read_bits(CURIE_IMU_RA_GYRO_RANGE,
                         CURIE_IMU_GYRO_RANGE_SEL_BIT,
                         CURIE_IMU_GYRO_RANGE_SEL_LEN);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
void BMI160Class::setGyroRange(int range) {
    reg_write_bits(CURIE_IMU_RA_GYRO_RANGE, range,
                   CURIE_IMU_GYRO_RANGE_SEL_BIT,
                   CURIE_IMU_GYRO_RANGE_SEL_LEN);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 *  3 = +/- 2g
 *  5 = +/- 4g
 *  8 = +/- 8g
 * 12 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see CURIE_IMU_RA_ACCEL_RANGE
 * @see BMI160AccelRange
 */
int BMI160Class::getAccelerometerRange() {
    return reg_read_bits(CURIE_IMU_RA_ACCEL_RANGE,
                         CURIE_IMU_ACCEL_RANGE_SEL_BIT,
                         CURIE_IMU_ACCEL_RANGE_SEL_LEN);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 * @see BMI160AccelRange
 */
void BMI160Class::setAccelerometerRange(int range) {
    reg_write_bits(CURIE_IMU_RA_ACCEL_RANGE, range,
                   CURIE_IMU_ACCEL_RANGE_SEL_BIT,
                   CURIE_IMU_ACCEL_RANGE_SEL_LEN);
}

/** Execute internal calibration to generate Accelerometer Axes offset value.
 * These can be retrieved using the getAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI160 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of 0g on the X-axis, the BMI160 device
 * must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target X/Y/Z-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getAccelOffset(int axis)
 * @see CURIE_IMU_RA_FOC_CONF
 * @see CURIE_IMU_RA_CMD
 */
void BMI160Class::autoCalibrateAccelerometerOffset(int axis, int target){
    int axisBit = 0;
    switch(axis){
        case X_AXIS:
            axisBit = CURIE_IMU_FOC_ACC_X_BIT;
            break;
        case Y_AXIS:
            axisBit = CURIE_IMU_FOC_ACC_Y_BIT;
            break;
        case Z_AXIS:
            axisBit = CURIE_IMU_FOC_ACC_Z_BIT;
            break;
        default:break;
    }
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << axisBit);
    else if (target == -1)
        foc_conf = (0x2 << axisBit);
    else if (target == 0)
        foc_conf = (0x3 << axisBit);
    else
        return;  /* Invalid target value */

    reg_write(CURIE_IMU_RA_FOC_CONF, foc_conf);
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_START_FOC);
    while (!(reg_read_bits(CURIE_IMU_RA_STATUS,
                           CURIE_IMU_STATUS_FOC_RDY,
                           1)))
        delay(1);
}


/** Get offset compensation value for accelerometer data.
 * Each axis accessed by entering X/Y/Z_AXIS as axis argument
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see CURIE_IMU_RA_OFFSET_0
 */
int BMI160Class::getAccelerometerOffset(int axis){
    switch(axis){
        case X_AXIS: return reg_read(CURIE_IMU_RA_OFFSET_0);
            break;
        case Y_AXIS: return reg_read(CURIE_IMU_RA_OFFSET_1);
            break;
        case Z_AXIS: return reg_read(CURIE_IMU_RA_OFFSET_2);
            break;
        default:    return 0;
            break;
    }
}

/** Set offset compensation value for accelerometer axes data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateAccelOffset().
 * @see getAccelOffset()
 * @see CURIE_IMU_RA_OFFSET_0-1-2.
 */
void BMI160Class::setAccelerometerOffset(int axis, int offset){
    switch(axis){
        case X_AXIS: reg_write(CURIE_IMU_RA_OFFSET_0, offset);
            break;
        case Y_AXIS: reg_write(CURIE_IMU_RA_OFFSET_1, offset);
            break;
        case Z_AXIS: reg_write(CURIE_IMU_RA_OFFSET_2, offset);
            break;
        default:
            break;
    }
    readAccelerometer(axis);
}

/** Set accel/gyro offset compensation enabled value.
 * @see getXAccelOffset()
 * @see CURIE_IMU_RA_OFFSET_6
 */

void BMI160Class::enableAccelerometerOffset(boolean enabled){
    reg_write_bits(CURIE_IMU_RA_OFFSET_6, enabled ? 0x01 : 0,
                  CURIE_IMU_ACC_OFFSET_EN,
                  1) ;
}

void BMI160Class::enableGyroOffset(boolean enabled){
    reg_write_bits(CURIE_IMU_RA_OFFSET_6, enabled ? 0x01 : 0,
                  CURIE_IMU_GYR_OFFSET_EN,
                  1) ;
}


/** Get gyro/accel offset compensation enabled value.
 * @see getXGyroOffset()
 * @see CURIE_IMU_RA_OFFSET_6
 */
boolean BMI160Class::accelerometerOffsetEnabled(){
    return !!(reg_read_bits(CURIE_IMU_RA_OFFSET_6,
                            CURIE_IMU_ACC_OFFSET_EN,
                            1));
}
boolean BMI160Class::gyroOffsetEnabled(){
    return !!(reg_read_bits(CURIE_IMU_RA_OFFSET_6,
                            CURIE_IMU_GYR_OFFSET_EN,
                            1));
}


/** Execute internal calibration to generate Gyro offset values.
 * This populates the Gyro offset compensation values for all 3 axes.
 * These can be retrieved using the get[X/Y/Z]GyroOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure that NO rotation of the BMI160 device
 * occurs while this auto-calibration process is active.
 *
 * To enable offset compensation, @see setGyroOffsetEnabled()
 * @see setGyroOffsetEnabled()
 * @see getXGyroOffset()
 * @see getYGyroOffset()
 * @see getZGyroOffset()
 * @see CURIE_IMU_RA_FOC_CONF
 * @see CURIE_IMU_RA_CMD
 */
void BMI160Class::autoCalibrateGyroOffset() {
    uint8_t foc_conf = (1 << CURIE_IMU_FOC_GYR_EN);
    reg_write(CURIE_IMU_RA_FOC_CONF, foc_conf);
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_START_FOC);
    while (!(reg_read_bits(CURIE_IMU_RA_STATUS,
                           CURIE_IMU_STATUS_FOC_RDY,
                           1)))
        delay(1);
}

/** Get offset compensation value for gyroscope X-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see CURIE_IMU_RA_OFFSET_3
 * @see CURIE_IMU_RA_OFFSET_6
 */
int BMI160Class::getGyroOffset(int axis){
    int offsetRegister = 0;
    int mostSignifBit = 0;
    switch(axis){
        case X_AXIS:    offsetRegister = CURIE_IMU_RA_OFFSET_3;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_X_MSB_BIT;
            break;
        case Y_AXIS:    offsetRegister = CURIE_IMU_RA_OFFSET_4;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_Y_MSB_BIT;
            break;
        case Z_AXIS:    offsetRegister = CURIE_IMU_RA_OFFSET_5;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_Z_MSB_BIT;
            break;
        default:    break;
    }
    int offset = reg_read(offsetRegister);
    offset |= (int)(reg_read_bits(CURIE_IMU_RA_OFFSET_6,
                                      mostSignifBit,
                                      CURIE_IMU_GYR_OFFSET_MSB_LEN)) << 8;
    return CURIE_IMU_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope data
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getXGyroOffset()
 * @see CURIE_IMU_RA_OFFSET_3-4-5
 * @see CURIE_IMU_RA_OFFSET_6
 */
void BMI160Class::setGyroOffset(int axis, int offset){
    int offsetRegister = 0;
    int mostSignifBit = 0;
    switch(axis){
        case X_AXIS:
            offsetRegister = CURIE_IMU_RA_OFFSET_3;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_X_MSB_BIT;
            break;
        case Y_AXIS:
            offsetRegister = CURIE_IMU_RA_OFFSET_4;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_Y_MSB_BIT;
            break;
        case Z_AXIS:
            offsetRegister = CURIE_IMU_RA_OFFSET_5;
            mostSignifBit = CURIE_IMU_GYR_OFFSET_Z_MSB_BIT;
            break;
    }
    reg_write(offsetRegister, offset);
    reg_write_bits(CURIE_IMU_RA_OFFSET_6, offset >> 8,
                   mostSignifBit,
                   CURIE_IMU_GYR_OFFSET_X_MSB_LEN);
    readGyro(axis); /* Read and discard the next data value */
}

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of int_low_th is 1LSB = 7.81mg (min: 3.91mg). Free Fall
 * is detected when the absolute value of the accelerometer measurements for the
 * three axes are each less than the detection threshold. This condition
 * triggers the Free-Fall (low-g) interrupt if the condition is maintained for
 * the duration specified in the int_low_dur field of the INT_LOWHIGH[0]
 * register (@see CURIE_IMU_RA_INT_LOWHIGH_0)
 *
 * For more details on the Free Fall detection interrupt, see Section 2.6.7 of the
 * BMI160 Data Sheet.
 *
 * @return Current free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
 * @see CURIE_IMU_RA_INT_LOWHIGH_1
 *
*/

/** Get shock event acceleration threshold.
 * This register configures the detection threshold for Shock event
 * detection. The unit of threshold is dependent on the accelerometer
 * sensitivity range (@see getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  7.81 mg/LSB (0 =  3.91mg)
 * +/- 4g           | 15.63 mg/LSB (0 =  7.81mg)
 * +/- 8g           | 31.25 mg/LSB (0 = 15.63mg)
 * +/- 16g          | 62.50 mg/LSB (0 = 31.25mg)
 * </pre>
 *
 * Shock is detected when the absolute value of the accelerometer measurements
 * for any of the three axes exceeds the detection threshold. This condition
 * triggers the Shock (high-g) interrupt if the condition is maintained without
 * a sign-change for the duration specified in the int_high_dur field of the
 * INT_LOWHIGH[3] register (@see CURIE_IMU_RA_INT_LOWHIGH_3).
 *
 * For more details on the Shock (high-g) detection interrupt, see Section 2.6.8 of the
 * BMI160 Data Sheet.
 *
 * @return Current shock acceleration threshold value
 * @see CURIE_IMU_RA_INT_LOWHIGH_4
 */

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation in the INT_MOTION[1] register. The unit of threshold is
 * dependent on the accelerometer sensitivity range (@see
 * getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  3.91 mg/LSB
 * +/- 4g           |  7.81 mg/LSB
 * +/- 8g           | 15.63 mg/LSB
 * +/- 16g          | 31.25 mg/LSB
 * </pre>
 *
 * Motion is detected when the difference between the absolute value of
 * consecutive accelerometer measurements for the 3 axes exceeds this Motion
 * detection threshold. This condition triggers the Motion interrupt if the
 * condition is maintained for the sample count interval specified in the
 * int_anym_dur field of the INT_MOTION[0] register (@see CURIE_IMU_RA_INT_MOTION_0)
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in INT_STATUS[2] (@see CURIE_IMU_RA_INT_STATUS_2).
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current motion detection acceleration threshold value
 * @see getMotionDetectionDuration()
 * @see CURIE_IMU_RA_INT_MOTION_1
 */

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation in the INT_MOTION[1] register. The unit of threshold is
 * dependent on the accelerometer sensitivity range
 * (@see getFullScaleAccelRange()) as follows:
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  3.91 mg/LSB
 * +/- 4g           |  7.81 mg/LSB
 * +/- 8g           | 15.63 mg/LSB
 * +/- 16g          | 31.25 mg/LSB
 * </pre>
 *
 * Zero Motion is detected when the difference between the value of
 * consecutive accelerometer measurements for each axis remains smaller than
 * this Motion detection threshold. This condition triggers the Zero Motion
 * interrupt if the condition is maintained for a time duration
 * specified in the int_slo_no_mot_dur field of the INT_MOTION[0] register (@see
 * CURIE_IMU_RA_INT_MOTION_0), and clears the interrupt when the condition is
 * then absent for the same duration.
 *
 * For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
 * the BMI160 Data Sheet.
 *
 * @return Current zero motion detection acceleration threshold value
 * @see getDetectionDuration()
 * @see CURIE_IMU_RA_INT_MOTION_2
 */

/** Get Tap event acceleration threshold.
 * This register configures the detection threshold for Tap event
 * detection. The threshold is expressed a 5-bit unsigned integer.
 * The unit of threshold is dependent on the accelerometer
 * sensitivity range (@see getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  62.5 mg/LSB (0 =  31.25mg)
 * +/- 4g           | 125.0 mg/LSB (0 =  62.5mg)
 * +/- 8g           | 250.0 mg/LSB (0 = 125.0mg)
 * +/- 16g          | 500.0 mg/LSB (0 = 250.0mg)
 * </pre>
 *
 * A Tap is detected as a shock event which exceeds the detection threshold for
 * a specified duration.  A threshold between 0.7g and 1.5g in the 2g
 * measurement range is suggested for typical tap detection applications.
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current shock acceleration threshold value
 * @see CURIE_IMU_RA_INT_TAP_1
 */
int BMI160Class::getDetectionThreshold(BMI160Feature feature){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            return reg_read(CURIE_IMU_RA_INT_LOWHIGH_1);
            break;
        case CURIE_IMU_SHOCK:
            return reg_read(CURIE_IMU_RA_INT_LOWHIGH_4);
            break;
        case CURIE_IMU_MOTION:
            return reg_read(CURIE_IMU_RA_INT_MOTION_1);
            break;
        case CURIE_IMU_ZERO_MOTION:
            return reg_read(CURIE_IMU_RA_INT_MOTION_2);
            break;
        case CURIE_IMU_TAP:
            return reg_read_bits(CURIE_IMU_RA_INT_TAP_1,
                         CURIE_IMU_TAP_THRESH_BIT,
                         CURIE_IMU_TAP_THRESH_LEN);
            break;
        default:
            return 0;
            break;
    }
}


/** Set free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
 * @see getDetectionThreshold()
 * @see CURIE_IMU_RA_INT_LOWHIGH_1
 */

/** Set shock event acceleration threshold.
 * @param threshold New shock acceleration threshold value
 * @see getShockDetectionThreshold()
 * @see CURIE_IMU_RA_INT_LOWHIGH_4
 */

/** Set motion detection event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value
 * @see getMotionDetectionThreshold()
 * @see CURIE_IMU_RA_INT_MOTION_1
 */


/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value
 * @see getZeroMotionDetectionThreshold()
 * @see CURIE_IMU_RA_INT_MOTION_2
 */


/** Set tap event acceleration threshold.
 * @param threshold New tap acceleration threshold value
 * @see getTapDetectionThreshold()
 * @see CURIE_IMU_RA_INT_TAP_1
*/
void BMI160Class::setDetectionThreshold(BMI160Feature feature, int threshold){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            reg_write(CURIE_IMU_RA_INT_LOWHIGH_1, threshold);
            break;
        case CURIE_IMU_SHOCK:
            reg_write(CURIE_IMU_RA_INT_LOWHIGH_4, threshold);
            break;
        case CURIE_IMU_MOTION:
            return reg_write(CURIE_IMU_RA_INT_MOTION_1, threshold);
            break;
        case CURIE_IMU_ZERO_MOTION:
            reg_write(CURIE_IMU_RA_INT_MOTION_2, threshold);
            break;
        case CURIE_IMU_TAP:
            reg_write_bits(CURIE_IMU_RA_INT_TAP_1, threshold,
                   CURIE_IMU_TAP_THRESH_BIT,
                   CURIE_IMU_TAP_THRESH_LEN);
            break;
        default:
            break;
    }
}

/** Get free-fall event duration threshold.
 * This register configures the duration threshold for Free Fall event
 * detection. The int_low_dur field of the INT_LOWHIGH[0] register has a unit
 * of 1 LSB = 2.5 ms (minimum 2.5ms).
 *
 * For more details on the Free Fall detection interrupt, see Section 2.6.7 of
 * the BMI160 Data Sheet.
 *
 * @return Current free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see CURIE_IMU_RA_INT_LOWHIGH_0
 */

/** Get shock event duration threshold.
 * This register configures the duration threshold for Shock event
 * detection. The int_high_dur field of the INT_LOWHIGH[3] register has a unit
 * of 1 LSB = 2.5 ms (minimum 2.5ms).
 *
 * For more details on the Shock detection interrupt, see Section 2.6.8 of
 * the BMI160 Data Sheet.
 *
 * @return Current shock duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see CURIE_IMU_RA_INT_LOWHIGH_3
 */

/** Set free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see getFreefallDetectionDuration()
 * @see CURIE_IMU_RA_INT_LOWHIGH_0
 */

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation, as a number of consecutive samples (from 1-4). The time
 * between samples depends on the accelerometer Output Data Rate
 * (@see getAccelRate()).
 *
 * The Motion detection interrupt is triggered when the difference between
 * samples exceeds the Any-Motion interrupt threshold for the number of
 * consecutive samples specified here.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current motion detection duration threshold value (#samples [1-4])
 * @see getMotionDetectionThreshold()
 * @see CURIE_IMU_RA_INT_MOTION_0
 */

/** Get zero motion detection event duration threshold.
 * This register configures the duration time for Zero Motion interrupt
 * generation. A time range between 1.28s and 430.08s can be selected, but the
 * granularity of the timing reduces as the duration increases:
 *
 * <pre>
 * Duration           | Granularity
 * -------------------+----------------
 * [1.28 - 20.48]s    |  1.28s
 * [25.6 - 102.4]s    |  5.12s
 * [112.64 - 430.08]s | 10.24s
 * </pre>
 *
 * The Zero Motion interrupt is triggered when the Zero Motion condition is
 * maintained for the duration specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
 * the BMI160 Data Sheet.
 *
 * @return Current zero motion detection duration threshold value
 *         @see BMI160ZeroMotionDuration for a list of possible values
 * @see getZeroMotionDetectionThreshold()
 * @see CURIE_IMU_RA_INT_MOTION_0
 * @see BMI160ZeroMotionDuration
 */

/** Get double-tap detection time window length.
 * This register configures the length of time window between 2 tap events for
 * double-tap event generation.
 *
 * The time will be returned as a 3-bit unsigned integer, with the following
 * values (@see BMI160DoubleTapDuration)
 *
 * <pre>
 * duration specifier | length of time window
 * -------------------+----------------
 *  0b000             |  50ms
 *  0b001             | 100ms
 *  0b010             | 150ms
 *  0b011             | 200ms
 *  0b100             | 250ms
 *  0b101             | 375ms
 *  0b110             | 500ms
 *  0b111             | 700ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current double-tap detection time window threshold value
 * @see CURIE_IMU_RA_INT_TAP_0
 * @see BMI160DoubleTapDuration
 */

/** Get tap quiet duration threshold.
 * This register configures the quiet duration for double-tap event detection.
 *
 * The time will be returned as a 1-bit boolean, with the following
 * values (@see BMI160TapQuietDuration)
 *
 * <pre>
 * duration specifier | duration threshold
 * -------------------+----------------
 *  0b0               |  30ms
 *  0b1               |  20ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current tap quiet detection duration threshold value
 * @see CURIE_IMU_RA_INT_TAP_0
 * @see BMI160TapQuietDuration
 */

/** Get tap shock detection duration.
 * This register configures the duration for a tap event generation.
 *
 * The time will be returned as a 1-bit boolean, with the following
 * values (@see BMI160TapShockDuration)
 *
 * <pre>
 * duration specifier | duration threshold
 * -------------------+----------------
 *  0b0               |  50ms
 *  0b1               |  75ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current tap detection duration threshold value
 * @see CURIE_IMU_RA_INT_TAP_0
 * @see BMI160TapShockDuration
 */

int BMI160Class::getDetectionDuration(BMI160Feature feature){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            return reg_read(CURIE_IMU_RA_INT_LOWHIGH_0);
            break;
        case CURIE_IMU_SHOCK:
            return reg_read(CURIE_IMU_RA_INT_LOWHIGH_3);
            break;
        case CURIE_IMU_MOTION:
            return 1 + reg_read_bits(CURIE_IMU_RA_INT_MOTION_0,
                             CURIE_IMU_ANYMOTION_DUR_BIT,
                             CURIE_IMU_ANYMOTION_DUR_LEN);
            break;
        case CURIE_IMU_ZERO_MOTION:
            return reg_read_bits(CURIE_IMU_RA_INT_MOTION_0,
                         CURIE_IMU_NOMOTION_DUR_BIT,
                         CURIE_IMU_NOMOTION_DUR_LEN);
            break;
        case CURIE_IMU_DOUBLE_TAP:
            return reg_read_bits(CURIE_IMU_RA_INT_TAP_0,
                         CURIE_IMU_TAP_DUR_BIT,
                         CURIE_IMU_TAP_DUR_LEN);
            break;
        case CURIE_IMU_TAP_QUIET:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_TAP_0,
                            CURIE_IMU_TAP_QUIET_BIT,
                            1));
            break;
        case CURIE_IMU_TAP_SHOCK:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_TAP_0,
                            CURIE_IMU_TAP_SHOCK_BIT,
                            1));
            break;
        default:
            return 0;
            break;
    }
}


/** Set free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see getFreefallDetectionDuration()
 * @see CURIE_IMU_RA_INT_LOWHIGH_3
 */

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (#samples [1-4])
 * @see getMotionDetectionDuration()
 * @see CURIE_IMU_RA_INT_MOTION_0
 */

/** Set zero motion detection event duration threshold.
 *
 * This must be called at least once to enable zero-motion detection.
 *
 * @param duration New zero motion detection duration threshold value
 *        @see BMI160ZeroMotionDuration for a list of valid values
 * @see getZeroMotionDetectionDuration()
 * @see CURIE_IMU_RA_INT_MOTION_0
 * @see BMI160ZeroMotionDuration
 */

/** Set tap shock detection event duration threshold.
 *
 * @param units New tap detection duration threshold value
 * @see getTapShockDetectionDuration()
 * @see CURIE_IMU_RA_INT_TAP_0
 */

/** Set tap quiet duration threshold.
 *
 * @param units New tap detection duration threshold value
 * @see getTapQuietDuration()
 * @see CURIE_IMU_RA_INT_TAP_0
 */

/** Set double-tap detection event duration threshold.
 *
 * @param duration New double-tap detection time window threshold value
 * @see getDoubleTapDetectionDuration()
 * @see CURIE_IMU_RA_INT_TAP_0
 */

void BMI160Class::setDetectionDuration(BMI160Feature feature, int value){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            reg_write(CURIE_IMU_RA_INT_LOWHIGH_0, value); //duration
            break;
        case CURIE_IMU_SHOCK:
            reg_write(CURIE_IMU_RA_INT_LOWHIGH_3, value); //duration
            break;
        case CURIE_IMU_MOTION:
            reg_write_bits(CURIE_IMU_RA_INT_MOTION_0, value - 1, //samples
                   CURIE_IMU_ANYMOTION_DUR_BIT,
                   CURIE_IMU_ANYMOTION_DUR_LEN);
            break;
        case CURIE_IMU_ZERO_MOTION:
            reg_write_bits(CURIE_IMU_RA_INT_MOTION_0, value, //duration
                   CURIE_IMU_NOMOTION_DUR_BIT,
                   CURIE_IMU_NOMOTION_DUR_LEN);
            break;
        case CURIE_IMU_DOUBLE_TAP:
            reg_write_bits(CURIE_IMU_RA_INT_TAP_0, value, //duration
                   CURIE_IMU_TAP_DUR_BIT,
                   CURIE_IMU_TAP_DUR_LEN);
            break;
        case CURIE_IMU_TAP_QUIET:
            reg_write_bits(CURIE_IMU_RA_INT_TAP_0, value ? 0x1 : 0, //bool duration
                   CURIE_IMU_TAP_QUIET_BIT,
                   1);
            break;
        case CURIE_IMU_TAP_SHOCK:
            reg_write_bits(CURIE_IMU_RA_INT_TAP_0, value ? 0x1 : 0, //bool duration
                   CURIE_IMU_TAP_SHOCK_BIT,
                   1);
            break;
        default:
            break;
    }
}

/////////////////////////////////STEP FUNCTIONS//////////////////////////////////

/** Get Step Detection mode.
 * Returns an enum value which corresponds to current mode
 * 0 = Normal Mode
 * 1 = Sensitive Mode
 * 2 = Robust Mode
 * 3 = Unkown Mode
 * For more details on the Step Detection, see Section
 * 2.11.37 of the BMI160 Data Sheet.
 *
 * @return Current configuration of the step detector
 * @see BMI160_RA_STEP_CONF_0
 * @see BMI160_RA_STEP_CONF_1
 */
BMI160StepMode BMI160Class::getStepDetectionMode() {
    uint8_t ret_step_conf0, ret_min_step_buf;

    ret_step_conf0 = reg_read(CURIE_IMU_RA_STEP_CONF_0);
    ret_min_step_buf = reg_read(CURIE_IMU_RA_STEP_CONF_1);

    if ((ret_step_conf0 == CURIE_IMU_RA_STEP_CONF_0_NOR) && (ret_min_step_buf == CURIE_IMU_RA_STEP_CONF_1_NOR))
        return CURIE_IMU_STEP_MODE_NORMAL;
    else if ((ret_step_conf0 == CURIE_IMU_RA_STEP_CONF_0_SEN) && (ret_min_step_buf == CURIE_IMU_RA_STEP_CONF_1_SEN))
       return CURIE_IMU_STEP_MODE_SENSITIVE;
    else if ((ret_step_conf0 == CURIE_IMU_RA_STEP_CONF_0_ROB) && (ret_min_step_buf == CURIE_IMU_RA_STEP_CONF_1_ROB))
        return CURIE_IMU_STEP_MODE_ROBUST;
    else
        return CURIE_IMU_STEP_MODE_UNKNOWN;
}

/** Set Step Detection mode.
 * Sets the step detection mode to one of 3 predefined sensitivity settings:
 *
 *  @see CURIE_IMU_STEP_MODE_NORMAL (Recommended for most applications)
 *  @see CURIE_IMU_STEP_MODE_SENSITIVE
 *  @see CURIE_IMU_STEP_MODE_ROBUST
 *
 * Please refer to Section 2.11.37 of the BMI160 Data Sheet for more information
 * on Step Detection configuration.
 *
 * @return Set Step Detection mode
 * @see CURIE_IMU_RA_STEP_CONF_0
 * @see CURIE_IMU_RA_STEP_CONF_1
 * @see BMI160StepMode
 */

void BMI160Class::setStepDetectionMode(BMI160StepMode mode) {
    uint8_t step_conf0, min_step_buf;

    /* Applying pre-defined values suggested in data-sheet Section 2.11.37 */
    switch (mode) {
    case CURIE_IMU_STEP_MODE_NORMAL:
        step_conf0 = 0x15;
        min_step_buf = 0x3;
        break;
    case CURIE_IMU_STEP_MODE_SENSITIVE:
        step_conf0 = 0x2D;
        min_step_buf = 0x0;
        break;
    case CURIE_IMU_STEP_MODE_ROBUST:
        step_conf0 = 0x1D;
        min_step_buf = 0x7;
        break;
    default:
        /* Unrecognised mode option */
        return;
    };
    reg_write(CURIE_IMU_RA_STEP_CONF_0, step_conf0);
    reg_write_bits(CURIE_IMU_RA_STEP_CONF_1, min_step_buf,
                   CURIE_IMU_STEP_BUF_MIN_BIT,
                   CURIE_IMU_STEP_BUF_MIN_LEN);
}

/** Get Step Counter enabled status.
 * Once enabled and configured correctly (@see setStepDetectionMode()), the
 * BMI160 will increment a counter for every step detected by the accelerometer.
 * To retrieve the current step count, @see getStepCount().
 *
 * For more details on the Step Counting feature, see Section
 * 2.7 of the BMI160 Data Sheet.
 *
 * @return Current Step Counter enabled status
 * @see CURIE_IMU_RA_STEP_CONF_1
 * @see CURIE_IMU_STEP_CNT_EN_BIT
 */
bool BMI160Class::getStepCountEnabled() {
    return !!(reg_read_bits(CURIE_IMU_RA_STEP_CONF_1,
                            CURIE_IMU_STEP_CNT_EN_BIT,
                            1));
}

/** Set Step Counter enabled status.
 *
 * @return Set Step Counter enabled
 * @see getStepCountEnabled()
 * @see CURIE_IMU_RA_STEP_CONF_1
 * @see CURIE_IMU_STEP_CNT_EN_BIT
 */
void BMI160Class::setStepCountEnabled(bool enabled) {
    return reg_write_bits(CURIE_IMU_RA_STEP_CONF_1, enabled ? 0x1 : 0,
                          CURIE_IMU_STEP_CNT_EN_BIT,
                          1);
}


/** Get current number of detected step movements (Step Count).
 * Returns a step counter which is incremented when step movements are detected
 * (assuming Step Detection mode and Step Counter are configured/enabled).
 *
 * @return Number of steps as an unsigned 16-bit integer
 * @see setStepCountEnabled()
 * @see setStepDetectionMode()
 * @see CURIE_IMU_RA_STEP_CNT_L
 */
long BMI160Class::getStepCount() {
    uint8_t buffer[2];
    buffer[0] = CURIE_IMU_RA_STEP_CNT_L;
    serial_buffer_transfer(buffer, 1, 2);
    return (((uint16_t)buffer[1]) << 8) | buffer[0];
}

/** Resets the current number of detected step movements (Step Count) to 0.
 *
 * @see getStepCount()
 * @see CURIE_IMU_RA_CMD
 */
void BMI160Class::resetStepCount() {
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_STEP_CNT_CLR);
}

/////////////////////////////////END OF STEP FUNCTIONS//////////////////////////////////

/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_LOW_G_EN_BIT
 **/

/** Get Shock interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_HIGH_G_EN_BIT
 **/

/** Get Step interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_2
 * @see CURIE_IMU_STEP_EN_BIT
 **/

/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_ANYMOTION_EN_BIT
 **/

/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_2
 * @see CURIE_IMU_NOMOTION_EN_BIT
 **/

/** Get Tap Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_S_TAP_EN_BIT
 **/
/** Get Tap Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_D_TAP_EN_BIT
 **/

/** Get FIFO Buffer Full interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_FFULL_EN_BIT
 **/

/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_DRDY_EN_BIT
 */


boolean BMI160Class::interruptEnabled(BMI160Feature feature){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_1,
                            CURIE_IMU_LOW_G_EN_BIT,
                            CURIE_IMU_LOW_G_EN_LEN));
        case CURIE_IMU_SHOCK:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_1,
                            CURIE_IMU_HIGH_G_EN_BIT,
                            CURIE_IMU_HIGH_G_EN_LEN));
            break;
        case CURIE_IMU_STEP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_2,
                            CURIE_IMU_STEP_EN_BIT,
                            1));
            break;
        case CURIE_IMU_MOTION:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_0,
                            CURIE_IMU_ANYMOTION_EN_BIT,
                            CURIE_IMU_ANYMOTION_EN_LEN));
            break;
        case CURIE_IMU_ZERO_MOTION:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_2,
                            CURIE_IMU_NOMOTION_EN_BIT,
                            CURIE_IMU_NOMOTION_EN_LEN));
            break;
        case CURIE_IMU_DOUBLE_TAP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_0,
                            CURIE_IMU_D_TAP_EN_BIT,
                            1));
            break;
        case CURIE_IMU_TAP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_0,
                            CURIE_IMU_S_TAP_EN_BIT,
                            1));
            break;
        case CURIE_IMU_DATA_READY:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_1,
                            CURIE_IMU_DRDY_EN_BIT,
                            1));
            break;
        case CURIE_IMU_FIFO_FULL:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_EN_1,
                            CURIE_IMU_FFULL_EN_BIT,
                            1));
            break;
        default:    return 0;
            break;
    }
}

/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_LOW_G_EN_BIT
 **/

/** Set Shock interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntShockEnabled()
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_HIGH_G_EN_BIT
 **/

/** Set Step interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntStepEnabled()
 * @see CURIE_IMU_RA_INT_EN_2
 * @see CURIE_IMU_STEP_EN_BIT
 **/

/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_ANYMOTION_EN_BIT
 **/

/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see CURIE_IMU_RA_INT_EN_2
 * @see CURIE_IMU_NOMOTION_EN_BIT
 * @see CURIE_IMU_RA_INT_MOTION_3
 **/

/** Set Tap Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntTapEnabled()
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_S_TAP_EN_BIT
 **/

/** Set Tap Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntTapEnabled()
 * @see CURIE_IMU_RA_INT_EN_0
 * @see CURIE_IMU_D_TAP_EN_BIT
 **/

/** Set FIFO Buffer Full interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferFullEnabled()
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_FFULL_EN_BIT
 **/

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see CURIE_IMU_RA_INT_EN_1
 * @see CURIE_IMU_DRDY_EN_BIT
 */
void BMI160Class::enableInterrupt(BMI160Feature feature, boolean enabled){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            reg_write_bits(CURIE_IMU_RA_INT_EN_1, enabled ? 0x1 : 0,
                   CURIE_IMU_LOW_G_EN_BIT,
                   CURIE_IMU_LOW_G_EN_LEN);
        case CURIE_IMU_SHOCK:
            reg_write_bits(CURIE_IMU_RA_INT_EN_1, enabled ? 0x7 : 0x0,
                   CURIE_IMU_HIGH_G_EN_BIT,
                   CURIE_IMU_HIGH_G_EN_LEN);
            break;
        case CURIE_IMU_STEP:
            reg_write_bits(CURIE_IMU_RA_INT_EN_2, enabled ? 0x1 : 0x0,
                   CURIE_IMU_STEP_EN_BIT,
                   1);
            break;
        case CURIE_IMU_MOTION:
            /* Enable for all 3 axes */
            reg_write_bits(CURIE_IMU_RA_INT_EN_0, enabled ? 0x7 : 0x0,
                   CURIE_IMU_ANYMOTION_EN_BIT,
                   CURIE_IMU_ANYMOTION_EN_LEN);
            break;
        case CURIE_IMU_ZERO_MOTION:
            if (enabled) {
                /* Select No-Motion detection mode */
                reg_write_bits(CURIE_IMU_RA_INT_MOTION_3, 0x1,
                       CURIE_IMU_NOMOTION_SEL_BIT,
                       CURIE_IMU_NOMOTION_SEL_LEN);
            }
            /* Enable for all 3 axes */
            reg_write_bits(CURIE_IMU_RA_INT_EN_2, enabled ? 0x7 : 0x0,
                   CURIE_IMU_NOMOTION_EN_BIT,
                   CURIE_IMU_NOMOTION_EN_LEN);
            break;
        case CURIE_IMU_DOUBLE_TAP:
            reg_write_bits(CURIE_IMU_RA_INT_EN_0, enabled ? 0x1 : 0,
                   CURIE_IMU_D_TAP_EN_BIT,
                   1);
            break;
        case CURIE_IMU_TAP:
            reg_write_bits(CURIE_IMU_RA_INT_EN_0, enabled ? 0x1 : 0,
                   CURIE_IMU_S_TAP_EN_BIT,
                   1);
            break;
        case CURIE_IMU_DATA_READY:
            reg_write_bits(CURIE_IMU_RA_INT_EN_1, enabled ? 0x1 : 0x0,
                   CURIE_IMU_DRDY_EN_BIT,
                   1);
            break;
        case CURIE_IMU_FIFO_FULL:
            reg_write_bits(CURIE_IMU_RA_INT_EN_1, enabled ? 0x1 : 0x0,
                   CURIE_IMU_FFULL_EN_BIT,
                   1);
            break;
        default:
            break;
    }
}

/////////////FIFO FUNCTIONS////////////////////

/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables accelerometer data samples to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 */
boolean BMI160Class::getAccelerometerFIFOEnabled() {
    return !!(reg_read_bits(CURIE_IMU_RA_FIFO_CONFIG_1,
                            CURIE_IMU_FIFO_ACC_EN_BIT,
                            1));
}

/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 */
void BMI160Class::setAccelerometerFIFOEnabled(bool enabled) {
    reg_write_bits(CURIE_IMU_RA_FIFO_CONFIG_1, enabled ? 0x1 : 0,
                   CURIE_IMU_FIFO_ACC_EN_BIT,
                   1);
}

/** Get gyroscope FIFO enabled value.
 * When set to 1, this bit enables gyroscope data samples to be
 * written into the FIFO buffer.
 * @return Current gyroscope FIFO enabled value
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 */
boolean BMI160Class::getGyroFIFOEnabled() {
    return !!(reg_read_bits(CURIE_IMU_RA_FIFO_CONFIG_1,
                            CURIE_IMU_FIFO_GYR_EN_BIT,
                            1));
}

/** Set gyroscope FIFO enabled value.
 * @param enabled New gyroscope FIFO enabled value
 * @see getGyroFIFOEnabled()
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 */
void BMI160Class::setGyroFIFOEnabled(bool enabled) {
    reg_write_bits(CURIE_IMU_RA_FIFO_CONFIG_1, enabled ? 0x1 : 0,
                   CURIE_IMU_FIFO_GYR_EN_BIT,
                   1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer.
 *
 * In "headerless" FIFO mode, it is directly proportional to the number of
 * samples available given the set of sensor data bound to be stored in the
 * FIFO. See @ref getFIFOHeaderModeEnabled().
 *
 * @return Current FIFO buffer size
 * @see CURIE_IMU_RA_FIFO_LENGTH_0
 */
long BMI160Class::getFIFOCount() {
    uint8_t buffer[2];
    buffer[0] = CURIE_IMU_RA_FIFO_LENGTH_0;
    serial_buffer_transfer(buffer, 1, 2);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Reset the FIFO.
 * This command clears all data in the FIFO buffer.  It is recommended
 * to invoke this after reconfiguring the FIFO.
 *
 * @see CURIE_IMU_RA_CMD
 * @see CURIE_IMU_CMD_FIFO_FLUSH
 */
void BMI160Class::resetFIFO() {
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_FIFO_FLUSH);
}

/** Reset the Interrupt controller.
 * This command clears interrupt status registers and latched interrupts.
 *
 * @see CURIE_IMU_RA_CMD
 * @see CURIE_IMU_CMD_FIFO_FLUSH
 */
void BMI160Class::resetInterrupt() {
    reg_write(CURIE_IMU_RA_CMD, CURIE_IMU_CMD_INT_RESET);
}

/** Get FIFO Header-Mode enabled status.
 * When this bit is set to 0, the FIFO header-mode is disabled, and frames
 * read from the FIFO will be headerless (raw sensor data only).
 * When this bit is set to 1, the FIFO header-mode is enabled, and frames
 * read from the FIFO will include headers.
 *
 * For more information on the FIFO modes and data formats, please refer
 * to Section 2.5 of the BMI160 Data Sheet.
 *
 * @return Current FIFO Header-Mode enabled status
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 * @see CURIE_IMU_FIFO_HEADER_EN_BIT
 */
boolean BMI160Class::getFIFOHeaderModeEnabled() {
    return !!(reg_read_bits(CURIE_IMU_RA_FIFO_CONFIG_1,
                            CURIE_IMU_FIFO_HEADER_EN_BIT,
                            1));
}

/** Set FIFO Header-Mode enabled status.
 * @param enabled New FIFO Header-Mode enabled status
 * @see getFIFOHeaderModeEnabled()
 * @see CURIE_IMU_RA_FIFO_CONFIG_1
 * @see CURIE_IMU_FIFO_HEADER_EN_BIT
 */
void BMI160Class::setFIFOHeaderModeEnabled(boolean enabled) {
    reg_write_bits(CURIE_IMU_RA_FIFO_CONFIG_1, enabled ? 0x1 : 0,
                   CURIE_IMU_FIFO_HEADER_EN_BIT,
                   1);
}

/** Get data frames from FIFO buffer.
 * This register is used to read and write data frames from the FIFO buffer.
 * Data is written to the FIFO in order of DATA register number (from lowest
 * to highest) corresponding to the FIFO data sources enabled (@see
 * getGyroFIFOEnabled() and getAccelFIFOEnabled()).
 *
 * The data frame format depends on the enabled data sources and also on
 * the FIFO header-mode setting (@see getFIFOHeaderModeEnabled()).
 *
 * It is strongly recommended, where possible, to read whole frames from the
 * FIFO.  Partially-read frames will be repeated until fully read out.
 *
 * If the FIFO buffer has filled to the point where subsequent writes may
 * cause data loss, the status bit ffull_int is automatically set to 1. This bit
 * is located in INT_STATUS[1]. When the FIFO buffer has overflowed, the oldest
 * data will be lost and new data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return a magic number
 * (@see CURIE_IMU_FIFO_DATA_INVALID) until new data is available. The user should
 * check FIFO_LENGTH to ensure that the FIFO buffer is not read when empty (see
 * @getFIFOCount()).
 *
 * @return Data frames from FIFO buffer
 */
void BMI160Class::getFIFOBytes(uint8_t *data, uint16_t length) {
    if (length) {
        data[0] = CURIE_IMU_RA_FIFO_DATA;
        serial_buffer_transfer(data, 1, length);
    }
}

//////////////////END OF FIFO FUNCTIONS////////////////////////

/** Get full set of interrupt status bits from INT_STATUS[] registers.
 * Interrupts are typically cleared automatically.
 * Please refer to the BMI160 Data Sheet for more information.
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0-4
 */

int BMI160Class::getInterruptBits(int reg){
    switch(reg){
        case 0: return reg_read(CURIE_IMU_RA_INT_STATUS_0);
            break;
        case 1: return reg_read(CURIE_IMU_RA_INT_STATUS_1);
            break;
        case 2: return reg_read(CURIE_IMU_RA_INT_STATUS_2);
            break;
        case 3: return reg_read(CURIE_IMU_RA_INT_STATUS_3);
            break;
        default: return 0;
            break;
    }
}

/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Free-Fall (Low-G) detection interrupt, see Section
 * 2.6.7 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_1
 * @see CURIE_IMU_LOW_G_INT_BIT
 */

/** Get Tap Detection interrupt status.
 * This bit automatically sets to 1 when a Tap Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0
 * @see CURIE_IMU_S_TAP_INT_BIT
 */

/** Get Double-Tap Detection interrupt status.
 * This bit automatically sets to 1 when a Double-Tap Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Double-Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0
 * @see CURIE_IMU_D_TAP_INT_BIT
 */

/** Get Shock interrupt status.
 * This bit automatically sets to 1 when a Shock (High-G) Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Shock (High-G) detection interrupt, see Section
 * 2.6.8 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_1
 * @see CURIE_IMU_HIGH_G_INT_BIT
 */

/** Get Step interrupt status.
 * This bit automatically sets to 1 when a Step Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Step detection interrupt, see Section
 * 2.6.3 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0
 * @see CURIE_IMU_STEP_INT_BIT
 */

/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0
 * @see CURIE_IMU_ANYMOTION_INT_BIT
 */

/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.9 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_1
 * @see CURIE_IMU_NOMOTION_INT_BIT
 */

/** Get FIFO Buffer Full interrupt status.
 * This bit automatically sets to 1 when a FIFO Full condition has been
 * generated. The bit clears to 0 when the FIFO is not full.
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_1
 * @see CURIE_IMU_FFULL_INT_BIT
 */

/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the data registers have been read.
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_1
 * @see CURIE_IMU_FFULL_INT_BIT
 */

int BMI160Class::getInterruptStatus(BMI160Feature feature){
    switch(feature){
        case CURIE_IMU_FREEFALL:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_1,
                            CURIE_IMU_LOW_G_INT_BIT,
                            1));
            break;
        case CURIE_IMU_SHOCK:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_1,
                            CURIE_IMU_HIGH_G_INT_BIT,
                            1));
            break;
        case CURIE_IMU_MOTION:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_0,
                            CURIE_IMU_ANYMOTION_INT_BIT,
                            1));
            break;
        case CURIE_IMU_ZERO_MOTION:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_1,
                            CURIE_IMU_NOMOTION_INT_BIT,
                            1));
            break;
        case CURIE_IMU_STEP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_0,
                            CURIE_IMU_STEP_INT_BIT,
                            1));
        case CURIE_IMU_TAP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_0,
                            CURIE_IMU_S_TAP_INT_BIT,
                            1));
            break;
        case CURIE_IMU_DOUBLE_TAP:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_0,
                            CURIE_IMU_D_TAP_INT_BIT,
                            1));
            break;
        case CURIE_IMU_FIFO_FULL:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_1,
                            CURIE_IMU_FFULL_INT_BIT,
                            1));
            break;
        case CURIE_IMU_DATA_READY:
            return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_1,
                            CURIE_IMU_DRDY_INT_BIT,
                            1));
            break;
        default: return 0;
            break;
    }
}

/** Check if shock interrupt was triggered by negative X-axis motion
 * @return Shock detection status
 * @see CURIE_IMU_RA_INT_STATUS_3
 * @see CURIE_IMU_HIGH_G_SIGN_BIT
 * @see CURIE_IMU_HIGH_G_1ST_X_BIT
 */
boolean BMI160Class::shockDetected(int axis, int direction){
    int axisBit = 0;
    switch(axis){
        case X_AXIS: axisBit = CURIE_IMU_HIGH_G_1ST_X_BIT;
            break;
        case Y_AXIS: axisBit = CURIE_IMU_HIGH_G_1ST_Y_BIT;
            break;
        case Z_AXIS: axisBit = CURIE_IMU_HIGH_G_1ST_Z_BIT;
            break;
        default: break;
    }
    uint8_t status = reg_read(CURIE_IMU_RA_INT_STATUS_3);
    if(direction == NEGATIVE){
        return !!((status & (1 << CURIE_IMU_HIGH_G_SIGN_BIT)) &&
                (status & (1 << axisBit)));
    }
    else {
        return !!(!(status & (1 << CURIE_IMU_HIGH_G_SIGN_BIT)) &&
              (status & (1 << axisBit)));
    }
}


/** Check if motion interrupt was triggered by negative X-axis motion
 * @return Motion detection status
 * @see CURIE_IMU_RA_INT_STATUS_2
 * @see CURIE_IMU_ANYMOTION_SIGN_BIT
 * @see CURIE_IMU_ANYMOTION_1ST_X_BIT
 */
boolean BMI160Class::motionDetected(int axis, int direction){
    int axisBit = 0;
    switch(axis){
        case X_AXIS: axisBit = CURIE_IMU_ANYMOTION_1ST_X_BIT;
            break;
        case Y_AXIS: axisBit = CURIE_IMU_ANYMOTION_1ST_Y_BIT;
            break;
        case Z_AXIS: axisBit = CURIE_IMU_ANYMOTION_1ST_Z_BIT;
            break;
        default: break;
    }
    uint8_t status = reg_read(CURIE_IMU_RA_INT_STATUS_2);
    if(direction == NEGATIVE){
        return !!((status & (1 << CURIE_IMU_ANYMOTION_SIGN_BIT)) &&
                (status & (1 << axisBit)));
    }
    else {
        return !!(!(status & (1 << CURIE_IMU_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << axisBit)));
    }
}


/** Check if tap interrupt was triggered in any axis or direction
 * @return Tap detection status
 * @see CURIE_IMU_RA_INT_STATUS_2
 * @see CURIE_IMU_TAP_SIGN_BIT
 * @see CURIE_IMU_TAP_1ST_X/Y/Z_BIT
 */
boolean BMI160Class::tapDetected(int axis, int direction){
    int axisBit = 0;
    switch(axis){
        case X_AXIS: axisBit = CURIE_IMU_TAP_1ST_X_BIT;
            break;
        case Y_AXIS: axisBit = CURIE_IMU_TAP_1ST_Y_BIT;
            break;
        case Z_AXIS: axisBit = CURIE_IMU_TAP_1ST_Z_BIT;
            break;
        default: break;
    }
    uint8_t status = reg_read(CURIE_IMU_RA_INT_STATUS_2);
    if(direction == NEGATIVE){
        return !!((status & (1 << CURIE_IMU_TAP_SIGN_BIT)) &&
                (status & (1 << axisBit)));
    }
    else {
        return !!(!(status & (1 << CURIE_IMU_TAP_SIGN_BIT)) &&
              (status & (1 << axisBit)));
    }
}

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see CURIE_IMU_RA_INT_OUT_CTRL
 * @see CURIE_IMU_INT1_LVL
 */
bool BMI160Class::getInterruptMode() {
    return !(reg_read_bits(CURIE_IMU_RA_INT_OUT_CTRL,
                           CURIE_IMU_INT1_LVL,
                           1));
}

/** Get Step interrupt status.
 * This bit automatically sets to 1 when a Step Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Step detection interrupt, see Section
 * 2.6.3 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see CURIE_IMU_RA_INT_STATUS_0
 * @see CURIE_IMU_STEP_INT_BIT
 */
bool BMI160Class::stepsDetected() {
    return !!(reg_read_bits(CURIE_IMU_RA_INT_STATUS_0,
                            CURIE_IMU_STEP_INT_BIT,
                            1));
}


/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see CURIE_IMU_RA_INT_OUT_CTRL
 * @see CURIE_IMU_INT1_LVL
 */
void BMI160Class::setInterruptMode(bool mode) {
    reg_write_bits(CURIE_IMU_RA_INT_OUT_CTRL, mode ? 0x0 : 0x1,
                   CURIE_IMU_INT1_LVL,
                   1);
}

/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see CURIE_IMU_RA_INT_OUT_CTRL
 * @see CURIE_IMU_INT1_OD
 */
bool BMI160Class::getInterruptDrive() {
    return !!(reg_read_bits(CURIE_IMU_RA_INT_OUT_CTRL,
                            CURIE_IMU_INT1_OD,
                            1));
}

/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
void BMI160Class::setInterruptDrive(bool drive) {
    reg_write_bits(CURIE_IMU_RA_INT_OUT_CTRL, drive ? 0x1 : 0x0,
                   CURIE_IMU_INT1_OD,
                   1);
}

/** Get interrupt latch mode.  The following options are available:
 *
 * <pre>
 * Latch Mode    | Interrupt Latching
 * --------------+-------------------------
 * 0             | non-latched
 * 1             | temporary, 312.5us pulse
 * 2             | temporary,   625us pulse
 * 3             | temporary,  1.25ms pulse
 * 4             | temporary,   2.5ms pulse
 * 5             | temporary,     5ms pulse
 * 6             | temporary,    10ms pulse
 * 7             | temporary,    20ms pulse
 * 8             | temporary,    40ms pulse
 * 9             | temporary,    80ms pulse
 * 10            | temporary,   160ms pulse
 * 11            | temporary,   320ms pulse
 * 12            | temporary,   640ms pulse
 * 13            | temporary,  1.28s pulse
 * 14            | temporary,  2.56s pulse
 * 15            | latched until cleared (@see resetInterrupt())
 * </pre>
 *
 * Note that latching does not apply to the following interrupt sources:
 * - Data Ready
 * - Orientation (including Flat) detection
 *
 * @return Current latch mode
 * @see CURIE_IMU_RA_INT_LATCH
 * @see BMI160InterruptLatchMode
 */
int BMI160Class::getInterruptLatch() {
    return reg_read_bits(CURIE_IMU_RA_INT_LATCH,
                         CURIE_IMU_LATCH_MODE_BIT,
                         CURIE_IMU_LATCH_MODE_LEN);
}

/** Set interrupt latch mode.
 * @param latch New latch mode
 * @see getInterruptLatch()
 * @see CURIE_IMU_RA_INT_LATCH
 * @see BMI160InterruptLatchMode
 */
void BMI160Class::setInterruptLatch(uint8_t mode) {
    reg_write_bits(CURIE_IMU_RA_INT_LATCH, mode,
                   CURIE_IMU_LATCH_MODE_BIT,
                   CURIE_IMU_LATCH_MODE_LEN);
}

/** Get interrupt enabled status.
 * @return Current interrupt enabled status
 * @see CURIE_IMU_RA_INT_OUT_CTRL
 * @see CURIE_IMU_INT1_OUTPUT_EN
 **/
bool BMI160Class::getIntEnabled() {
    return !!(reg_read_bits(CURIE_IMU_RA_INT_OUT_CTRL,
                            CURIE_IMU_INT1_OUTPUT_EN,
                            1));
}

/** Set interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see CURIE_IMU_RA_INT_OUT_CTRL
 * @see CURIE_IMU_INT1_OUTPUT_EN
 **/
void BMI160Class::setIntEnabled(bool enabled) {
    reg_write_bits(CURIE_IMU_RA_INT_OUT_CTRL, enabled ? 0x1 : 0,
                   CURIE_IMU_INT1_OUTPUT_EN,
                   1);
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
 * @see CURIE_IMU_RA_GYRO_X_L
 */
void BMI160Class::readMotionSensor(short& ax, short& ay, short& az, short& gx, short& gy, short& gz) {
    uint8_t buffer[12];
    buffer[0] = CURIE_IMU_RA_GYRO_X_L;
    serial_buffer_transfer(buffer, 1, 12);
    gx = (((short)buffer[1])  << 8) | buffer[0];
    gy = (((short)buffer[3])  << 8) | buffer[2];
    gz = (((short)buffer[5])  << 8) | buffer[4];
    ax = (((short)buffer[7])  << 8) | buffer[6];
    ay = (((short)buffer[9])  << 8) | buffer[8];
    az = (((short)buffer[11]) << 8) | buffer[10];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Output Data Rate
 * as configured by @see getAccelRate()
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale configured by
 * @setFullScaleAccelRange. For each full scale setting, the accelerometers'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range | LSB Sensitivity
 * -----------------+----------------
 * +/- 2g           | 8192 LSB/mg
 * +/- 4g           | 4096 LSB/mg
 * +/- 8g           | 2048 LSB/mg
 * +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see CURIE_IMU_RA_ACCEL_X_L
 */
void BMI160Class::readAcceleration(short& x, short& y, short& z) {
    uint8_t buffer[6];
    buffer[0] = CURIE_IMU_RA_ACCEL_X_L;
    serial_buffer_transfer(buffer, 1, 6);
    x = (((short)buffer[1]) << 8) | buffer[0];
    y = (((short)buffer[3]) << 8) | buffer[2];
    z = (((short)buffer[5]) << 8) | buffer[4];
}

/** Get accelerometer reading.
 * @return axes' acceleration measurement in 32-bit signed format
 * @see readMotionSensor()
 * @see CURIE_IMU_RA_ACCEL_X_L
 */

int BMI160Class::readAccelerometer(int axis){
    int accelAxis = 0;
    switch(axis){
        case X_AXIS: accelAxis = CURIE_IMU_RA_ACCEL_X_L;
            break;
        case Y_AXIS: accelAxis = CURIE_IMU_RA_ACCEL_Y_L;
            break;
        case Z_AXIS: accelAxis = CURIE_IMU_RA_ACCEL_Z_L;
            break;
    }
    uint8_t buffer[2];
    buffer[0] = accelAxis;
    serial_buffer_transfer(buffer, 1, 2);
    return (((short)buffer[1]) << 8) | buffer[0];
}

/** Get current internal temperature as a signed 16-bit integer.
 *  The resolution is typically 1/2^9 degrees Celcius per LSB, at an
 *  offset of 23 degrees Celcius.  For example:
 *
 * <pre>
 * Value    | Temperature
 * ---------+----------------
 * 0x7FFF   | 87 - 1/2^9 degrees C
 * ...      | ...
 * 0x0000   | 23 degrees C
 * ...      | ...
 * 0x8001   | -41 + 1/2^9 degrees C
 * 0x8000   | Invalid
 *
 * @return Temperature reading in 16-bit 2's complement format
 * @see CURIE_IMU_RA_TEMP_L
 */
int BMI160Class::readTemperature() {
    uint8_t buffer[2];
    buffer[0] = CURIE_IMU_RA_TEMP_L;
    serial_buffer_transfer(buffer, 1, 2);
    return (((short)buffer[1]) << 8) | buffer[0];
}

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale configured by
 * @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range   | LSB Sensitivity
 * -------------------+----------------
 * +/- 125  degrees/s | 262.4 LSB/deg/s
 * +/- 250  degrees/s | 131.2 LSB/deg/s
 * +/- 500  degrees/s | 65.5  LSB/deg/s
 * +/- 1000 degrees/s | 32.8  LSB/deg/s
 * +/- 2000 degrees/s | 16.4  LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see readMotionSensor()
 * @see CURIE_IMU_RA_GYRO_X_L
 */
void BMI160Class::readRotation(short& x, short& y, short& z) {
    uint8_t buffer[6];
    buffer[0] = CURIE_IMU_RA_GYRO_X_L;
    serial_buffer_transfer(buffer, 1, 6);
    x = (((short)buffer[1]) << 8) | buffer[0];
    y = (((short)buffer[3]) << 8) | buffer[2];
    z = (((short)buffer[5]) << 8) | buffer[4];
}

/** Get axes' gyroscope reading.
 * @return rotation measurement in 32-bit signed format
 * @see readMotionSensor()
 * @see CURIE_IMU_RA_GYRO_X_L
 */
int BMI160Class::readGyro(int axis){
    int gyroAxis = 0;
    switch(axis){
        case(X_AXIS):   gyroAxis = CURIE_IMU_RA_GYRO_X_L;
            break;
        case(Y_AXIS):   gyroAxis = CURIE_IMU_RA_GYRO_Y_L;
            break;
        case(Z_AXIS):   gyroAxis = CURIE_IMU_RA_GYRO_Z_L;
            break;
    }
    uint8_t buffer[2];
    buffer[0] = gyroAxis;
    serial_buffer_transfer(buffer, 1, 2);
    return (((short)buffer[1]) << 8) | buffer[0];
}

/** Read a BMI160 register directly.
 * @param reg register address
 * @return 8-bit register value
 */
int BMI160Class::getRegister(int reg) {
    return reg_read(reg);
}

/** Write a BMI160 register directly.
 * @param reg register address
 * @param data 8-bit register value
 */
void BMI160Class::setRegister(int reg, int data) {
    reg_write(reg, data);
}
