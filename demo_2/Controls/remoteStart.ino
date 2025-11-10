// Remote Start Arduino Code
// Primary developer: Kiera Crawford
// Updated: 10/22/2025
// Description: This is a helper file that just does the comms part of Arduino tasks
// This is a comms testing file but the code can be copied to match functionalty in main sketch

#include <Wire.h>

#define MY_ADDR 0x08

float received_distance = 0.0;
float received_rotation = 0.0;
bool received = false;

void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceiveEvent);
  Wire.onRequest(onRequestEvent);
  Serial.begin(115200);
  Serial.println("Ready to receive float motion commands.");
}

void loop() {
  if (received) {
    Serial.print("Received Distance (ft): ");
    Serial.print(received_distance, 2);
    Serial.print(" | Rotation (deg): ");
    Serial.println(received_rotation, 2);
    received = false;
  }
}

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

void onRequestEvent() {
  union { byte b[4]; float f; } dist, rot;
  dist.f = received_distance;
  rot.f = received_rotation;

  for (int i = 0; i < 4; i++) Wire.write(dist.b[i]);
  for (int i = 0; i < 4; i++) Wire.write(rot.b[i]);
}