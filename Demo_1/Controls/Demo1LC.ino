#define Motor_Enable 4
#define M1Voltage_Sign 7
#define M2Voltage_Sign 8
#define M1PWM 9
#define M2PWM 10
#define M1EncA 2
#define M2EncA 3
#define M1EncB 5
#define M2EncB 6
#define MY_ADDR 0x08  // Arduino's I2C address
#include <Wire.h>

volatile uint8_t number;
volatile uint8_t reply;
volatile bool received = false;

/*
Measured Motor Parameters:
K1 = 1.5  sigma1 = 15.75
K2 = 1.5  sigma2 = 14.5
*/

const float pi = 3.1415926538;      // pi with 8 decimal places
const float R = 0.074;              // Wheel radius in meters
const float wheel_base = 0.3556;    // Distance between wheels in meters
const int counts_per_rev = 3200; //number of encoder counts per full wheel revolution
const float BATTERY_VOLTAGE = 7.8;
const float ANGLE_TOLERANCE = 0.75 * (pi/180); // 0.75 degrees in radians
const float DISTANCE_TOLERANCE = 0.75 * 0.0254; // 0.75 inches in meters
float applied_voltage[2] = {0,0};
float PWM[2];

// Define gain, speed, and error variables
float Kp_vel[2] = {2,2};
float Kp_pos[2] = {15,15};
float Ki_pos[2] = {3,3};

float desired_pos_xy[2] = {0,0}; // {angle (degrees), distance (meters)}
bool angle_set = false; // Checks whether angle has been set and robot should move forward
bool angle_changed = false;
bool distance_changed = false;
float desired_pos_r[2] = {0,0}; // radians
volatile uint8_t return_vals[2] = {0, 0};
float desired_vel[2];   //rad/s

float actual_pos[2];   //radians
float last_pos[2] = {0,0};  //radians
float actual_vel[2];  //rad/s

float pos_error[2];
float integral_error[2];
float vel_error[2];

// Define volatile counter variables for motor encoders
volatile long int Enc_Counter[2] = {0,0};

//Odometry Variables
float posX = 0.0, posY = 0.0, phi = 0.0;
float prev_rad[2] = {0, 0}; // Previous wheel angles (radians)

// Define time keeping variables
unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
bool finished = false; // Define bool to use to print finished



void setup() {


  //Initialize wire
  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceiveEvent);
  Wire.onRequest(onRequestEvent);


  // Initialize motor pins as outputs
  pinMode(Motor_Enable, OUTPUT);
  pinMode(M1Voltage_Sign, OUTPUT);
  pinMode(M2Voltage_Sign, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  // Initialize encoder pins as inputs
  pinMode(M1EncA, INPUT);
  pinMode(M2EncA, INPUT);
  pinMode(M1EncB, INPUT);
  pinMode(M2EncB, INPUT);

  // Enable the motor
  digitalWrite(Motor_Enable, HIGH);

  Serial.begin(115200); // Set the baud rate fast so that we can display the results
  last_time_ms = millis(); // Set up sample time variable
  start_time_ms = last_time_ms;

  // Define interrupt functions for encoders
  attachInterrupt(digitalPinToInterrupt(M1EncA), M1Enc_Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), M2Enc_Update, CHANGE);

  // Print ready for MATLAB
  // Serial.println("Ready!");

}

void loop() {

    if (received){ // This doesnt really do anything but print random stuff you should put your code in this if statement, 
    noInterrupts();
    Serial.print("Received: ");
    Serial.print(desired_pos_r[0]);
    Serial.print(", ");
    Serial.println(desired_pos_r[1]);
    interrupts();
    received = false;
    }

    // Change desired position at 5 seconds
    if (current_time >= 5 && !finished) {
      desired_pos_xy[0] = 90;
      desired_pos_xy[1] = 0;
      angle_set = false;
      angle_changed = true;
      distance_changed = true;
      finished = true;
    }

    // Move robot to desired angle
    if (!angle_set) { // If the robot has not moved to the desired angle

      if (angle_changed) {
        // Calculations to convert desired robot angle to desired wheel angle
        desired_pos_r[0] += -( (desired_pos_xy[0]*(pi/180)) * wheel_base ) / (2*R);
        desired_pos_r[1] += ( (desired_pos_xy[0]*(pi/180)) * wheel_base ) / (2*R);
        angle_changed = false;
      }

      // Testing
      Serial.print("X Position: ");
      Serial.print(posX);
      Serial.print("\t");
      Serial.print("Y Position: ");
      Serial.print(posY);
      Serial.print("\t");
      Serial.print("Phi: ");
      Serial.print(phi);
      Serial.print("\t");
      Serial.print("Distance Travelled: ");
      Serial.println(sqrt( posX^2 + posY^2 ));

      // If the robots angle is within the desired tolerance, allow robot to move desired distance
      if ( (abs(phi - (desired_pos_xy[0]*(pi/180))) < ANGLE_TOLERANCE) && actual_vel[0] <  0.1) {
        angle_set = true;
        Serial.println("Angle Set");
      }

    } else if (angle_set) { 

      if (distance_changed) {
        // calculations to convert desired robot distance to desired wheel angle
        desired_pos_r[0] += desired_pos_xy[1] / R;
        desired_pos_r[1] += desired_pos_xy[1] / R;
        distance_changed = false;
      }

    }

    unsigned long now = millis();
    //if (now - last_time_ms < desired_Ts_ms) return;
    float dt = (now-last_time_ms)/1000.0f; //Change in time, bc it's not always exactly 10
    current_time = (float)(now-start_time_ms)/1000;    // Update current_time

    float wheel_rad[2];
    float delta_M[2]; //delta distance for each wheel

    for(int i=0;i<2;i++) {

      // Update motor positions
      actual_pos[i] = 2*pi*(float)Enc_Counter[i]/counts_per_rev;

      // Calculate motor velocity
      actual_vel[i] = (actual_pos[i] - last_pos[i]) / dt;

      // Calculate position and integral errors
      pos_error[i] = desired_pos_r[i] - actual_pos[i];
      
      integral_error[i] = integral_error[i] + pos_error[i]*(dt);
      integral_error[i] = constrain(integral_error[i], -10.0, 10.0); // CHANGE CONSTRAINTS

      // Calculate desired velocity and velocity error 
      desired_vel[i] = constrain(Kp_pos[i] * pos_error[i] + Ki_pos[i] * integral_error[i], -10, 10); // Constrain maximum desired veolcity to prevent slipping?
      vel_error[i] = desired_vel[i] - actual_vel[i];

      // Calculate applied voltage
      applied_voltage[i] = Kp_vel[i]*vel_error[i];

      // Calculate PWM signal 
      PWM[i] = constrain( (abs(applied_voltage[i]) / BATTERY_VOLTAGE) * 255, 0, 255);
      
      // Save for next iteration
      last_pos[i] = actual_pos[i];

      // For odometry
      wheel_rad[i] = actual_pos[i];
      delta_M[i] = (wheel_rad[i] - prev_rad[i]) * R;
      prev_rad[i] = wheel_rad[i];

    }

    // Set motor driver sign pins
    if (applied_voltage[0] > 0) { digitalWrite(M1Voltage_Sign, HIGH); }  
    else { digitalWrite(M1Voltage_Sign, LOW); }
    if (applied_voltage[1] > 0) { digitalWrite(M2Voltage_Sign, LOW); }  
    else { digitalWrite(M2Voltage_Sign, HIGH); }

    // Write PWM signals to motors
    analogWrite(M1PWM, PWM[0]);
    analogWrite(M2PWM, PWM[1]);

    //for odometry
    float delta_center = (delta_M[0] + delta_M[1]) / 2.0;
    phi += (delta_M[1] - delta_M[0]) / wheel_base;

    // Normalize phi
    if (phi > pi) phi -= 2 * pi;
    if (phi < -pi) phi += 2 * pi;

    posX += cos(phi) * delta_center;
    posY += sin(phi) * delta_center;

    /*
    // Record data
    if (current_time <= 100) {
      // Print time, voltage, position, and velocity
      Serial.print(current_time);
      Serial.print("\t");
      Serial.print(constrain(applied_voltage[0],-7.5,7.5));
      Serial.print("\t");
      Serial.print(actual_pos[0]);
      Serial.print("\t");
      Serial.print(actual_vel[0]);
      Serial.print("\t");
      Serial.print(posX); 
      Serial.print("\t");
      Serial.print(posY); 
      Serial.print("\t");
      Serial.println(phi*180/pi);
    } else if (current_time > 100) { 
      // Print finished for MATLAB
      if (!finished) {
        Serial.println("Finished");
      }
      finished = true;
    }
    */

    while (millis()<last_time_ms + desired_Ts_ms) {
    // Wait until desired time passes to go top of the loop
    } //Line at top replaces this??

    last_time_ms = now;  
}




void M1Enc_Update() { // Interrupt function for motor 1's encoder

  // Read encoder pins
  int A = digitalRead(M1EncA);
  int B = digitalRead(M1EncB);

  // Increment count based on direction of rotation
  if (A == B) { Enc_Counter[0] += 2; }
  else { Enc_Counter[0] -= 2; }

}

void M2Enc_Update() { // Interrupt function for motor 2's encoder

  // Read encoder pins
  int A = digitalRead(M2EncA);
  int B = digitalRead(M2EncB);

  // Increment count based on direction of rotation
  if (A == B) { Enc_Counter[1] -= 2; }
  else { Enc_Counter[1] += 2; }

}


void onReceiveEvent() {
  int i = 0;
  while (Wire.available() && i < 2){
    desired_pos_r[i] = Wire.read();
    return_vals[i] = desired_pos_r[i];
    i++;
  }
  received = true;
}

// This on a request event writes to the pi
void onRequestEvent() {
  Wire.write((const uint8_t*)return_vals, 2);        // send one byte
}