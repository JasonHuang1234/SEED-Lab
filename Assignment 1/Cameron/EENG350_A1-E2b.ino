+/* 
EENG350 - SEED Lab
Assignment 1 - Intro to Arduino
Exercise 2b - Encoder Reading With Interrupts

The following code tracks the movement of a rotary encoder
and increments a corresponding counter by using 1 interrupt.
*/

// Declare encoder pins
const int APIN = 2;
const int BPIN = 3;

// Declare volatile counter to be updated in ISR
volatile long enc_counter = 0;

void setup() {
  
  // Initialze serial communications
  Serial.begin(9600);

  // Initialize encoder pins as inputs
  pinMode(APIN, INPUT);
  pinMode(BPIN, INPUT);

  // Create interrupt for the A/CLK encoder pin
  attachInterrupt(digitalPinToInterrupt(APIN), encoder_update, CHANGE);

}

void loop() {
  
  // Initialize variable to store last printed count
  static long last_count = 0;
  
  // Print count only if it has changed
  if (last_count != enc_counter) {
    Serial.print("Count: ");
    Serial.println(enc_counter);
    last_count = enc_counter;
  }

}

void encoder_update() {

  // Read encoder pins
  int A = digitalRead(APIN);
  int B = digitalRead(BPIN);

  // Increment count based on direction of rotation
  if (A == B) { enc_counter -= 2; }
  else { enc_counter += 2; }

}
