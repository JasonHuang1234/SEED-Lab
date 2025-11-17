/* 
EENG350 - SEED Lab
Assignment 1 - Intro to Arduino
Exercise 1 - Blinking LED demo

The following sketch turns on/off 4 LEDs in order, with two push buttons
which increase/decrease the delay between LEDs changing state.
*/

// Declare each LED/Button pin
const int RED_PIN = 7;  
const int ORANGE_PIN = 6;
const int YELLOW_PIN = 5;
const int GREEN_PIN = 4;

const int GAS_PIN = 2;
const int BRAKE_PIN = 3;

// Declare a volatile delay variable that can be changed in an ISR
volatile int light_delay = 100;

void setup() {

  // Initialize the LED/Button pins as outputs/inputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(ORANGE_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  pinMode(GAS_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);

  // Create interrupts for each button pin
  attachInterrupt(digitalPinToInterrupt(GAS_PIN), gas, FALLING);
  attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), brake, FALLING);

}

void loop() {

  // Cycle each LED pin between on and off with a delay corresponding to light_delay
  digitalWrite(RED_PIN, HIGH);
  delay(light_delay);
  digitalWrite(ORANGE_PIN, HIGH);
  delay(light_delay);
  digitalWrite(YELLOW_PIN, HIGH);
  delay(light_delay);
  digitalWrite(GREEN_PIN, HIGH);
  delay(light_delay);

  digitalWrite(RED_PIN, LOW);
  delay(light_delay);
  digitalWrite(ORANGE_PIN, LOW);
  delay(light_delay);
  digitalWrite(YELLOW_PIN, LOW);
  delay(light_delay);
  digitalWrite(GREEN_PIN, LOW);
  delay(light_delay);

}

// Interrupt function to speed up the blinking rate
void gas() {

  if (light_delay > 10) { // Create minimum delay
    light_delay -= 10;
  }

}

// Interrupt function to slow down the blinking rate
void brake() {
  
  if (light_delay < 1000) { // Create maximum delay 
    light_delay += 10;
  }

}