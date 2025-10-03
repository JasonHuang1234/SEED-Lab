volatile int goal_M1 = 0;
volatile int goal_M2 = 0;

// input output vals
volatile uint8_t number;
volatile uint8_t reply;
MY_ADDR 0x08;

void setup(){
  Serial.begin(115200);
  Wire.begin(MY_ADDR);           // join I2C bus as slave once
  Wire.onReceive(onReceiveEvent);
  // Checks to see if 
  Wire.onRequest(onRequestEvent);
}




void loop() {
  static unsigned long last_time_ms = 0;
  if (received){ // This doesnt really do anything but print random stuff you should put your code in this if statement, 
    Serial.print("Got: "); Serial.print(number);
    Serial.print("Set goal_M1 as:  "); Serial.print(goal_M1);
    Serial.print("Set goal_M2 as:  "); Serial.print(goal_M2)
    Serial.print("  -> Reply: "); Serial.println(reply);
    digitalWrite(LED_BUILTIN, number & 0x01);  // just to show activity

  // !!!!!! Set received to false so we can exit this loop

    received = false;
  } // end of if received put to end of code

}

void onReceiveEvent(int nbytes) {
  while (Wire.available()) {
    number = Wire.read();
  }
  switch number:
    case 0:
      goal_M1 = 0;
      goal_M2 = 0;
    case 1:
      goal_M1 = 1;
      goal_M2 = 0;
    case 2:
      goal_M1 = 1;
      goal_M2 = 1;
    case 3:
      goal_M1 = 0;
      goal_M2 = 1;
  
  reply = (uint8_t)(number);
  received = true;
}

// This on a request event writes to the pi
void onRequestEvent() {
  Wire.write(reply);        // send one byte
}