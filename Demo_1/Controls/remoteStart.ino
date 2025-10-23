#include "RemoteStart.h"

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
  if (numBytes >= 8) {
    union { byte b[4]; float f; } dist, rot;
    for (int i = 0; i < 4; i++) dist.b[i] = Wire.read();
    for (int i = 0; i < 4; i++) rot.b[i] = Wire.read();
    received_distance = dist.f;
    received_rotation = rot.f;
    received = true;
  }
}

void onRequestEvent() {
  union { byte b[4]; float f; } dist, rot;
  dist.f = received_distance;
  rot.f = received_rotation;
  for (int i = 0; i < 4; i++) Wire.write(dist.b[i]);
  for (int i = 0; i < 4; i++) Wire.write(rot.b[i]);
}