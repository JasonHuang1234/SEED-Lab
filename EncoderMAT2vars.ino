#include <Wire.h>


// Constants
const float pi = 3.14159265;     // pi with 8 decimal places
const float R = 0.074;           // Wheel radius in meters
const float wheel_base = 0.3556;    // Distance between wheels in meters

// Encoder resolution
const int counts_per_rev = 3200; //number of encoder counts per full wheel revolution

// Odometry state
float posM1_rad = 0.0, posM2_rad = 0.0; //angular position of L and R wheels
float posM1_m = 0.0, posM2_m = 0.0; //Linear position of L and R wheels
float delta_M1 = 0.0, delta_M2 = 0.0; //Change in distance for L and R
float delta_center = 0.0; //Avg movement of the robots center
float phi = 0.0;                 // Orientation in radians of robot
float posX = 0.0, posY = 0.0;    // Position in meters of robot

unsigned long start_time_ms; //Timestamp of when program starts

// Encoder Pins
const int M1EncA = 2; //Encoder 1 Channel A  M1EncA
const int M1EncB = 5; //1B M1EncB
const int M2EncA = 3; //2A M2EncA
const int M2EncB = 6; //2B M2EncB

//For the number of encoder ticks for each motor
volatile long M1Enc_Counter = 0; //for left wheel encoder
volatile long M2Enc_Counter = 0; //for right wheel encoder


//Goal position (NE:00, NW:01, SW:11, SE:10)
volatile int start_M1 = 0;
volatile int start_M2 = 0;
volatile int goal_M1 = 0;
volatile int goal_M2 = 0;


// input output vals
volatile uint8_t number;
volatile uint8_t reply;
MY_ADDR 0x08;

// 

void setup() {
  //Set encoder pins as input
  pinMode(M1EncA, INPUT);
  pinMode(M1EncB, INPUT);
  pinMode(M2EncA, INPUT);
  pinMode(M2EncB, INPUT);

  //attach interrupts to encoder channels for tick counting
  attachInterrupt(digitalPinToInterrupt(M1EncA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), encoder2ISR, CHANGE);

  //record start time
  start_time_ms = millis();
  //initalize serial communication for data output with specified baud rate
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
    Serial.print("  -> Reply: "); Serial.println(reply);
    digitalWrite(LED_BUILTIN, number & 0x01);  // just to show activity

  // !!!!!! Set received to false so we can exit this loop

    received = false;
  } // end of if received put to end of code
 
  //For testing purposes, manually set goal positions 
  // goal_M1 = 1; //0 or 1
  // goal_M2 = 1; //0 or 1
  
  if (goal_M1 != start_M1 | goal_M2 != start_M2) {
    Serial.print("Goal Position: ");
    Serial.print(goal_M1), Serial.print("\t"), Serial.print(goal_M2);
    //Motor control to move 180 degrees for M1
    if (goal_M1 == 0) {
      // Move motor to spin wheel to 0 counts
    } else if (goal_M1 == 1) {
      // Move motor to spin wheel to 1600 counts
    }

    if (goal_M2 == 0) {
      // Move motor to spin wheel to 0 counts
    } else if (goal_M2 == 1) {
      // Move motor to spin wheel to 1600 counts
    }

    //Update start positions
    start_M1 = goal_M1;
    start_M2 = goal_M2;
  }


  //NE
  //M1Enc_Counter = 0 , M2Enc_Counter = 0;
  //NW
  //M1Enc_Counter = 0 , M2Enc_Counter = 1600;
  //SW
  //M1Enc_Counter = 1600 , M2Enc_Counter = 1600;
  //SE
  //M1Enc_Counter = 1600 , M2Enc_Counter = 0;


  if (millis() - last_time_ms >= 25) { // Update every 25 ms
    last_time_ms = millis();

    // 1) Read encoder counts and convert encoder ticks to radians
    float M1_rad = 2 * pi * (float)M1Enc_Counter / counts_per_rev;
    float M2_rad = 2 * pi * (float)M2Enc_Counter / counts_per_rev;
    
  
    // 2) Convert to distance moved by each wheel since last update (25ms)
    delta_M1 = (M1_rad - posM1_rad) * R;
    delta_M2 = -(M2_rad - posM2_rad) * R; //negative sign since motors are put in opposite directions
    
    
    //update linear positions of the wheels
    posM1_m += delta_M1;
    posM2_m += delta_M2;
   
    //update new wheel positions
    posM1_rad = M1_rad;
    posM2_rad = M2_rad;

    // 3) Compute center point movement of the robot's axis
    delta_center = (delta_M1 + delta_M2) / 2.0;

    // 4) Compute change in orientation based on difference of wheel movement
    phi += (delta_M2 - delta_M1) / wheel_base;

    // Normalize phi range to [-pi, pi]
    if (phi > pi) phi -= 2 * pi;
    if (phi < -pi) phi += 2 * pi;

    // 5) Update position of robot
    posX += cos(phi) * delta_center;
    posY += sin(phi) * delta_center;
    
    // 6) Compute current time
    float current_time = (millis() - start_time_ms) / 1000.0;

    // 7) Print values used for MATLAB
    Serial.print(current_time); Serial.print("\t");
    Serial.print(posX); Serial.print("\t");
    Serial.print(posY); Serial.print("\t");
    Serial.print(phi); Serial.print("\t");
    Serial.print(delta_M1, 6); Serial.print("\t");
    Serial.print(delta_M2, 6); Serial.print("\t");
    Serial.println(delta_center, 6);
  }
}

//ISRs for encoders
void encoder1ISR() {
  //Read encoder channels to find direction
  bool A = digitalRead(M1EncA);
  bool B = digitalRead(M1EncB);
  //Increment or decrement tick count depending on direction
  if (A == B) M1Enc_Counter+=2;
  else M1Enc_Counter-=2;
}

void encoder2ISR() {
  bool A = digitalRead(M2EncA);
  bool B = digitalRead(M2EncB);
  if (A == B) M2Enc_Counter+=2;
  else M2Enc_Counter-=2;
}




// Takes in one byte from the pi, from the arduino, the arduino saves the number as a reply, sets the goals and sets a received event to true.
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

