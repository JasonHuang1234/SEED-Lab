// Remote Start C++ Code
// Primary developer: Kiera Crawford
// 10/15/2025
// Description: Follower code for remote start. Recieves target positions, 
// these must be updated externally in if(recieved)

#include <Arduino.h>
#include <Wire.h>
#include "RemoteStart.h"

#define MY_ADDR 0x08

// Shared variables
volatile float received_distance = 0.0;
volatile float received_rotation = 0.0;
volatile bool received = false;

// I2C receive and request handlers
void onReceiveEvent(int numBytes) {
  if (numBytes >= 8) {
    union { byte b[4]; float f; } dist, rot;

    // Read 4 bytes for distance
    for (int i = 0; i < 4; i++) {
      if (Wire.available()) dist.b[i] = Wire.read();
    }

    // Read 4 bytes for rotation
    for (int i = 0; i < 4; i++) {
      if (Wire.available()) rot.b[i] = Wire.read();
    }

    received_distance = dist.f;
    received_rotation = rot.f;
    received = true;
  }
}

void onRequestEvent() {
  union { byte b[4]; float f; } dist, rot;
  dist.f = received_distance;
  rot.f = received_rotation;

  // Send both floats back
  for (int i = 0; i < 4; i++) Wire.write(dist.b[i]);
  for (int i = 0; i < 4; i++) Wire.write(rot.b[i]);
}