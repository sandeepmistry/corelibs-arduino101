/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect shocks or sudden movements
*/

#include "CurieImu.h"

boolean blinkState = false;          // state of the LED

void setup() {
  Serial.begin(9600);

  /* Initialise the IMU */
  CurieImu.begin();
  CurieImu.attachInterrupt(eventCallback);

  /* Enable Shock Detection */
  CurieImu.setDetectionThreshold(CURIE_IMU_SHOCK, 192); // 1.5g
  CurieImu.setDetectionDuration(CURIE_IMU_SHOCK,11);   // 30ms
  CurieImu.enableInterrupt(CURIE_IMU_SHOCK,true);

  /* Enable Interrupts Notifications */
  CurieImu.enableInts(true);

  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  // blink the LED in the main loop:
  digitalWrite(13, blinkState);
  blinkState = !blinkState;
  delay(1000);
}


static void eventCallback(void)
{
  if (CurieImu.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if (CurieImu.shockDetected(X_AXIS,POSITIVE))
      Serial.println("Negative shock detected on X-axis");
    if (CurieImu.shockDetected(X_AXIS,NEGATIVE))
      Serial.println("Positive shock detected on X-axis");
    if (CurieImu.shockDetected(Y_AXIS,POSITIVE))
      Serial.println("Negative shock detected on Y-axis");
    if (CurieImu.shockDetected(Y_AXIS,NEGATIVE))
      Serial.println("Positive shock detected on Y-axis");
    if (CurieImu.shockDetected(Z_AXIS,POSITIVE))
      Serial.println("Negative shock detected on Z-axis");
    if (CurieImu.shockDetected(Z_AXIS,NEGATIVE))
      Serial.println("Positive shock detected on Z-axis");
  }
}
