/*
Objective: Track the position and direction of a 
rotary encoder using interrupts and without interrupts
Stephanie Murphy
*/

#define APIN 2
#define BPIN 3

//defining variables used in ISR and read in main code
volatile int encoderCount = 0;  //encoder position
volatile int lastA = HIGH;      //last recorded state of A
volatile int lastB = HIGH;      //last recorded state of B
volatile unsigned long lastInterruptTime = 0; //time of last interrupt

unsigned long desired_Ts_ms = 25; //desired sampling period (25 ms)
unsigned long last_time_ms = 0;   //time of last sample

void setup() {
  Serial.begin(9600);
  //Configure encoder pins as inputs with internal pull-up resistors
  pinMode(APIN, INPUT_PULLUP);
  pinMode(BPIN, INPUT_PULLUP);
  //set interrupt onto digital pin 2, triggering encoderISR on state changes
  attachInterrupt(digitalPinToInterrupt(APIN), encoderISR, CHANGE);
}

void loop() {
  static int lastReportedCount = 0;
  //Sample the encoder state
  if (millis() - last_time_ms >= desired_Ts_ms) {
    int currentCount = MyEnc();
    //print if the count has changed
    if (currentCount != lastReportedCount) {
      Serial.print("Sampled Count: ");
      Serial.println(currentCount);
      lastReportedCount = currentCount;
    }
    last_time_ms = millis(); //update time
  }
}

//ISR for changes on APIN
void encoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime < 50) return; // debounce, where it ignores interrupts <50ms after last
  //read current states
  int thisA = digitalRead(APIN);
  int thisB = digitalRead(BPIN);

  // Determine direction of rotation
  if (thisA == thisB) {
    encoderCount += 2; //CW
  } else {
    encoderCount -= 2; //CCW
  }

  // Update states and time
  lastA = thisA;
  lastB = thisB;
  lastInterruptTime = currentTime;

  // Print count during rotation (because interrupt)
  Serial.print("ISR Count: ");
  Serial.println(encoderCount);
}

// Non-interrput sample-based encoder reader
int MyEnc() {
  //read current states
  int thisA = digitalRead(APIN);
  int thisB = digitalRead(BPIN);

  int adjustment = 0;

  // See if channel was changed
  if (thisA != lastA || thisB != lastB) {
    if (thisA == thisB) {
      adjustment = 1; //CW
    } else {
      adjustment = -1; //CCW
    }

    //update states
    lastA = thisA;
    lastB = thisB;
  }

  return encoderCount + adjustment; //return adjusted count
}