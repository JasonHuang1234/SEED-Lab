// Constants
const float Pi = 3.14159265;     // Pi with 8 decimal places
const float R = 0.074;           // Wheel radius in meters
const float wheel_base = 0.3556;    // Distance between wheels in meters

// Encoder resolution
const int counts_per_rev = 3200; //number of encoder counts per full wheel revolution

// Odometry state
float posL_rad = 0.0, posR_rad = 0.0; //angular position of L and R wheels
float posL_m = 0.0, posR_m = 0.0; //Linear position of L and R wheels
float delta_L = 0.0, delta_R = 0.0; //Change in distance for L and R
float delta_center = 0.0; //Avg movement of the robots center
float phi = 0.0;                 // Orientation in radians of robot
float posX = 0.0, posY = 0.0;    // Position in meters of robot

unsigned long start_time_ms; //Timestamp of when program starts

// Encoder Pins
const int enc1_A = 2; //Encoder 1 Channel A
const int enc1_B = 5; //1B
const int enc2_A = 3; //2A
const int enc2_B = 6; //2B

//For the number of encoder ticks for each motor
volatile long enc1_count = 0; //for left wheel encoder
volatile long enc2_count = 0; //for right wheel encoder

void setup() {
  //Set encoder pins as input
  pinMode(enc1_A, INPUT);
  pinMode(enc1_B, INPUT);
  pinMode(enc2_A, INPUT);
  pinMode(enc2_B, INPUT);

  //attach interrupts to encoder channels for tick counting
  attachInterrupt(digitalPinToInterrupt(enc1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2_A), encoder2ISR, CHANGE);

  //record start time
  start_time_ms = millis();
  //initalize serial communication for data output with specified baud rate
  Serial.begin(115200);
}

void loop() {
  static unsigned long last_time_ms = 0;
  
  if (millis() - last_time_ms >= 25) { // Update every 25 ms
    last_time_ms = millis();

    // 1) Read encoder counts and convert encoder ticks to radians
    noInterrupts();
    long enc1 = enc1_count;
    long enc2 = enc2_count;
    interrupts();

    float newL_rad = 2 * Pi * (float)enc1 / counts_per_rev;
    float newR_rad = 2 * Pi * (float)enc2 / counts_per_rev;

    // 2) Convert to distance moved by each wheel since last update (25ms)
    delta_L = (newL_rad - posL_rad) * R;
    delta_R = -(newR_rad - posR_rad) * R; //negative sign since motors are put in opposite directions
    
     //update linear positions of the wheels
    posL_m += delta_L;
    posR_m += delta_R;


    //update new wheel positions
    posL_rad = newL_rad;
    posR_rad = newR_rad;

    // 3) Compute center point movement of the robot's axis
    delta_center = (delta_L + delta_R) / 2.0;

    // 4) Compute change in orientation based on difference of wheel movement
    phi += (delta_R - delta_L) / wheel_base;

    // Normalize phi range to [-Pi, Pi]
    if (phi > Pi) phi -= 2 * Pi;
    if (phi < -Pi) phi += 2 * Pi;

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
    Serial.print(delta_L, 6); Serial.print("\t");
    Serial.print(delta_R, 6); Serial.print("\t");
    Serial.println(delta_center, 6);
  }
}

//ISRs for encoders
void encoder1ISR() {
  //Read encoder channels to find direction
  bool A = digitalRead(enc1_A);
  bool B = digitalRead(enc1_B);
  //Increment or decrement tick count depending on direction
  if (A == B) enc1_count+=2;
  else enc1_count-=2;
}

void encoder2ISR() {
  bool A = digitalRead(enc2_A);
  bool B = digitalRead(enc2_B);
  if (A == B) enc2_count+=2;
  else enc2_count-=2;
}
