#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;

//Sets baud rate and initializes wire
void setup() {
  Serial.begin(115200);
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
}

void loop() {
  if (msgLength > 0) {
    printReceived();
    msgLength = 0;
  }
}

// Display function for arduino serial
void printReceived() {
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.println("Characters and ASCII codes:");
  for (int i = 0; i < msgLength; i++) {
    char c = (char)instruction[i];
    Serial.print("Char: ");
    Serial.print(c);
    Serial.print(" | ASCII: ");
    Serial.println(instruction[i]);
  }
  Serial.println("");
}

// interupt for main loop
void receive(int numBytes) {
  if (numBytes < 1) return;
  offset = Wire.read(); // First byte is offset
  msgLength = 0;
  // reads chaacters
  while (Wire.available() && msgLength < sizeof(instruction)) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}