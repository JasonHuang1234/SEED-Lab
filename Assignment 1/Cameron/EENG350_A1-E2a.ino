/* 
EENG350 - SEED Lab
Assignment 1 - Intro to Arduino
Exercise 2a - Encoder Reading Without Interrupts

The following code tracks the movement of a rotary encoder
and increments a corresponding counter without using interrupts.
*/

// Declare encoder pins
const int APIN = 2;
const int BPIN = 3;

// Declare counter variable
long enc_counter = 0;

void setup() {
  
  // Initialize serial communications
  Serial.begin(9600);

  // Initialize encoder pins as inputs
  pinMode(APIN, INPUT);
  pinMode(BPIN, INPUT);

}

void loop() {
  
  // Read encoder pins
  int thisA = digitalRead(APIN);
  int thisB = digitalRead(BPIN);

  // Declare variables to store previous state
  static int lastA = 0;
  static int lastB = 0;
  
  // Check if encoder values have changed
  if (thisA != lastA || thisB != lastB) {
    
    // Increment counter based on direction of rotation
    if (lastA == lastB) {
      if (thisA == lastA) { enc_counter--; }
      else { enc_counter++; }
    }

    else {
      if (thisA == lastA) { enc_counter++; }
      else { enc_counter--; }
    }

    // Print encoder state and count
    Serial.print(thisA);
    Serial.print("\t");
    Serial.print(thisB);
    Serial.print("\t");;
    Serial.print("Count: ");
    Serial.print(enc_counter);
    Serial.print("\n");

    // Set previous state to current state
    lastA = thisA;
    lastB = thisB;

  }

}
