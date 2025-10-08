#include <Wire.h>

// input output vals
volatile uint8_t input[2];
volatile bool received = false;


#define MY_ADDR 0x08

void setup(){
  Serial.begin(115200);
  Wire.begin(MY_ADDR);           // join I2C bus as slave once
  Wire.onReceive(onReceiveEvent);
  // Checks to see if 
  Wire.onRequest(onRequestEvent);
}




void loop() {

  if (received){ // This doesnt really do anything but print random stuff you should put your code in this if statement, 
    noInterrupts();
    Serial.print("Received: ");
    Serial.print(input[0]);
    Serial.print(", ");
    Serial.print(input[1]);
    interrupts();
  // !!!!!! Set received to false so we can exit this loop

    received = false;
  } // end of if received put to end of code

}

void onReceiveEvent(int nbytes) {
  int i = 0;
  while (Wire.available() && i < 2){
    input[i] = Wire.read();
    i++;
  }
  received = true;
}

// This on a request event writes to the pi
void onRequestEvent() {
  Wire.write((const uint8_t*)input, 2);        // send one byte
}