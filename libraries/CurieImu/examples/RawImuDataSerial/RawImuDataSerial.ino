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
short int ax, ay, az;         // accelerometer values
short int gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

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

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieImu.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieImu.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieImu.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieImu.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieImu.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieImu.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieImu.setAccelerometerOffset(X_AXIS,128);
    //CurieImu.setAccelerometerOffset(Y_AXIS,-4);
    //CurieImu.setAccelerometerOffset(Z_AXIS,127);
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
    CurieImu.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieImu.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieImu.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieImu.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieImu.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieImu.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieImu.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieImu.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieImu.getGyroOffset(Z_AXIS));

    Serial.println("Enabling Gyroscope/Acceleration offset compensation");
    CurieImu.enableGyroOffset(false);
    CurieImu.enableAccelerometerOffset(true);

    Serial.println(CurieImu.accelerometerOffsetEnabled());
    Serial.println(CurieImu.gyroOffsetEnabled());
  }
  
  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  CurieImu.readMotionSensor(ax, ay, az, gx, gy, gz);

  // these methods (and a few others) are also available

  //CurieImu.readAcceleration(ax, ay, az);
  //CurieImu.readRotation(gx, gy, gz);

  //ax = CurieImu.readAccelerometer(X_AXIS);
  //ay = CurieImu.readAccelerometer(Y_AXIS);
  //az = CurieImu.readAccelerometer(Z_AXIS);
  //gx = CurieImu.readGyro(X_AXIS);
  //gy = CurieImu.readGyro(Y_AXIS);
  //gz = CurieImu.readGyro(Z_AXIS);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(ledPin, blinkState);
}