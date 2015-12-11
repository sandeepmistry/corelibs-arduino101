/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

/*
 * This sketch example demonstrates how the BMI160 accelerometer on the
 * Intel(R) Curie(TM) module can be used to detect tap events
 */

#include "CurieImu.h"

void setup() {
    Serial.begin(9600);

    // Initialise the IMU
    CurieImu.begin();
    CurieImu.attachInterrupt(eventCallback);

    // Increase Accelerometer range to allow detection of stronger taps (< 4g)
    CurieImu.setAccelRange(CURIE_IMU_ACCEL_RANGE_4G);

    // Reduce threshold to allow detection of weaker taps (>= 750mg)
    CurieImu.setDetectionThreshold(CURIE_IMU_TAP,6); // (6 x 125mg)

    // Set the time window for 2 taps to be registered as a double-tap (<= 250 milliseconds)
    CurieImu.setDetectionDuration(CURIE_IMU_DOUBLE_TAP,CURIE_IMU_DOUBLE_TAP_DURATION_250MS);

    // Enable Double-Tap detection
    CurieImu.enableInterrupt(CURIE_IMU_DOUBLE_TAP,true);

    // Enable Interrupts Notifications
    CurieImu.enableInts(true);

    Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  // nothing happens in the loop because all the action happens
  // in the callback function. 
}

static void eventCallback()
{
    if (CurieImu.getInterruptStatus(CURIE_IMU_DOUBLE_TAP)) {
     if (CurieImu.tapDetected(X_AXIS,NEGATIVE))
        Serial.println("Double Tap detected on negative X-axis");
     if (CurieImu.tapDetected(X_AXIS,POSITIVE))
        Serial.println("Double Tap detected on positive X-axis");
     if (CurieImu.tapDetected(Y_AXIS,NEGATIVE))
        Serial.println("Double Tap detected on negative Y-axis");
     if (CurieImu.tapDetected(Y_AXIS,POSITIVE))
        Serial.println("Double Tap detected on positive Y-axis");
     if (CurieImu.tapDetected(Z_AXIS,NEGATIVE))
        Serial.println("Double Tap detected on negative Z-axis");
     if (CurieImu.tapDetected(Z_AXIS,POSITIVE))
        Serial.println("Double Tap detected on positive Z-axis");
  }
}
