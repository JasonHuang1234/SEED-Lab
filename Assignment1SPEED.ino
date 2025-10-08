/*
Objective: Demonstrate speed control of sequencing LEDs where the 'gas'
speeds up the LED blinks and the 'brake' slows it down. It uses an ISR to 
help implement the changing of speed
Stephanie Murphy
*/


#define GAS_PIN 2
#define BRAKE_PIN 3
const int ledPins[] = {4, 5, 6, 7}; //Array of LED pins

// Shared delay variable controlling LED blink speed (in milliseconds)
// Modified by interrupt routines to simulate acceleration and braking
volatile int blinkDelay = 500;  // Initial delay in ms

void setup() {
  Serial.begin(9600); 
  // Set LED pins as OUTPUT
  for (int i = 0; i < 4; i++) pinMode(ledPins[i], OUTPUT);

  // Set button pins as INPUT with internal pull-up resistors
  pinMode(GAS_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);

  // Attach interrupt routines to GAS and BRAKE buttons
  // Trigger on FALLING edge (button press)
  attachInterrupt(digitalPinToInterrupt(GAS_PIN), speedUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), slowDown, FALLING);
}


void loop() {
  // Light up LEDs one by one
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(blinkDelay);
  }

  // Turn off LEDs one by one
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledPins[i], LOW);
    delay(blinkDelay);
  }
}

// ISR for GAS button
// Decreases delay to speed up LED sequence

void speedUp() {
  blinkDelay = max(50, blinkDelay - 50);
  Serial.println("Speeding up");
}

// ISR for BRAKE button
// Increases delay to slow down LED sequence
void slowDown() {
  blinkDelay = min(1000, blinkDelay + 50);
  Serial.println("Slowing down");
}