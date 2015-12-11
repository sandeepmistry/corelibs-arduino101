/*
  ===============================================
  Example sketch for CurieImu library for Intel(R) Curie(TM) devices.
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
  class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

  ===============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

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

#include "CurieImu.h"
int16_t xAccel,yAccel,zAccel;
int16_t xGyro,yGyro,zGyro;
int16_t ax, ay, az;         // accelerometer values
int16_t gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieImu.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieImu.begin()) {
    Serial.println("CurieImu connection successful");
  } else {
    Serial.println("CurieImu connection failed");
  }

  CurieImu.setGyroRate(CURIE_IMU_GYRO_RATE_25HZ);
  Serial.println(CurieImu.getGyroRate());

  CurieImu.setAccelFilterMode(CURIE_IMU_DLPF_MODE_NORM);
  Serial.println(CurieImu.getAccelFilterMode());

  CurieImu.setGyroFIFOEnabled(true);
  Serial.println(CurieImu.getGyroFIFOEnabled());

  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieImu.getAccelOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getAccelOffset(Y_AXIS));
  Serial.print("\t"); // -235
  Serial.print(CurieImu.getAccelOffset(Z_AXIS));
  Serial.print("\t"); // 168
  Serial.print(CurieImu.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getGyroOffset(Z_AXIS));

  // To manually configure offset compensation values,
  // use the following methods instead of the autoCalibrate...() methods below
      //CurieImu.setAccelOffset(X_AXIS,128);
      //CurieImu.setAccelOffset(Y_AXIS,-4);
      //CurieImu.setAccelOffset(Z_AXIS,127);
      //CurieImu.setGyroOffset(X_AXIS,129);
      //CurieImu.setGyroOffset(Y_AXIS,-1);
      //CurieImu.setGyroOffset(Z_AXIS, 254);
  
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(5000);

  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateAccelOffset(X_AXIS, 0);
  CurieImu.autoCalibrateAccelOffset(Y_AXIS, 0);
  CurieImu.autoCalibrateAccelOffset(Z_AXIS, 1);
  Serial.println(" Done");
  
  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieImu.getAccelOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getAccelOffset(Y_AXIS));
  Serial.print("\t"); // -2359
  Serial.print(CurieImu.getAccelOffset(Z_AXIS));
  Serial.print("\t"); // 1688
  Serial.print(CurieImu.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getGyroOffset(Z_AXIS));

  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.enableGyroOffset(false);
  CurieImu.enableAccelOffset(true);

  Serial.println(CurieImu.accelOffsetEnabled());
  Serial.println(CurieImu.gyroOffsetEnabled());

  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  CurieImu.readMotionSensor(ax, ay, az, gx, gy, gz);

  // these methods (and a few others) are also available
  //CurieImu.readAccelerometer(xAccel,yAccel,zAccel);
  //CurieImu.readGyroscope(xGyro, yGyro, zGyro);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  //Serial.print(xAccel);
  Serial.print("\t");
  Serial.print(ay);
  //Serial.print(yAccel);
  Serial.print("\t");
  Serial.print(az);
  //Serial.print(zAccel);
  Serial.print("\t");
  Serial.print(gx);
  //Serial.print(xGyro);
  Serial.print("\t");
  Serial.print(gy);
  //Serial.print(yGyro);
  Serial.print("\t");
  Serial.println(gz);
  //Serial.print(zGyro);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(ledPin, blinkState);
}
