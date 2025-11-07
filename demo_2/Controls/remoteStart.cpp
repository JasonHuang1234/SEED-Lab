// Remote Start C++ Code
// Primary developer: Kiera Crawford
// Updated: 10/22/2025
// Description: Handles I2C communication from Raspberry Pi, parsing remote control commands
// and determining what to do next. The main loop can use these globals to react.

#include <Arduino.h>
#include <Wire.h>
#include "RemoteStart.h"

#define MY_ADDR 0x08

// Shared variables (accessible to main sketch)
volatile float received_distance = 0.0;
volatile float received_rotation = 0.0;
volatile bool received = false;
volatile uint8_t command = 0x00;

// Flags to be used by main control logic
bool angle_changed = false;
bool distance_changed = false;

// External control targets (these are defined in main)
extern float desired_pos_xy[2];

// I2C receive handler
void onReceiveEvent(int numBytes) {
  if (numBytes >= 9) {
    union { byte b[4]; float f; } dist, rot;

    // Read command byte
    if (Wire.available()) {
      command = Wire.read();
    }

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

    // Decide what to do based on the command
    switch (command) {
      case 0x00: // optional
        continuous_turn = true;
        Serial.println("Resuming continuous rotation");
        break;

      case 0x01:
        continuous_turn = false;
        Serial.println("Stopping continuous rotation");
        break;

      case 0x02:
        continuous_turn = false;
        desired_pos_xy[0] = received_rotation;
        desired_pos_xy[1] = received_distance;
        Serial.println("Switching to Pi-guided movement");
        break;

      case 0x03:
        Serial.println("Adjusting left turn");
        desired_pos_xy[0] = -90;
        break;

      case 0x04:
        Serial.println("Adjusting right turn");
        desired_pos_xy[0] = 90;
        break;
    }
  }
}

// I2C request handler
void onRequestEvent() {
  union { byte b[4]; float f; } dist, rot;
  dist.f = received_distance;
  rot.f = received_rotation;

  for (int i = 0; i < 4; i++) Wire.write(dist.b[i]);
  for (int i = 0; i < 4; i++) Wire.write(rot.b[i]);
}
