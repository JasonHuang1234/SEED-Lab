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

float received_distance = 0.0;
float received_rotation = 0.0;
bool received = false;
//bool spinning = true;
byte command = 0;

const float pi = 3.1415926538;      // pi with 8 decimal places
const float WHEEL_RADIUS = 0.075;              // Wheel radius in meters
const float WHEEL_BASE = 0.356;    // Distance between wheels in meters
const int counts_per_rev = 3200; //number of encoder counts per full wheel revolution
const float BATTERY_VOLTAGE = 7.8;
const float ANGLE_TOLERANCE = 5 * (pi/180); // 5 degrees in radians
const float DISTANCE_TOLERANCE = 2 * 0.0254; // 1 inch in meters

float applied_voltage_rho[2] = {0,0}, applied_voltage_phi[2] = {0,0};
float PWM[2], desired_PWM[2];

// Gain variables for proportional and PI controllers
float Kp_vel[2] = {3,3.4};
float Kp_rho = 1, Kp_phi = 2;
float Ki_rho = 0.05, Ki_phi = 0.2;

float actual_vel[2] = {0,0}; // Motor velocity in radians/sec

float rho_error, rho_integral_error; // Meters
float phi_error, act_phi_error, phi_integral_error; // Radians
float x_error, y_error; // Meters
int direction;

volatile long int Enc_Counter[2] = {0,0}; // Counter variables for motor encoders

//Odometry Variables
float robot_position[2] = {0,0}; // Robot position in meters {x,y}
float desired_pos_xy[2] = {0,0}; // Desired robot position in meters {x,y}
float phi = 0; // Robot angle in radians
float desired_phi = 0; // Desired robot angle in radians
float desired_robot_vel = 0,  desired_robot_omega = 0;
float wheel_rad[2] = {0,0}; // Current wheel angles in radians
float prev_rad[2] = {0,0}; // Previous wheel angles in radians
float delta_M[2]; // Change in distance for each wheel

// Time keeping variables
unsigned long sample_time = 3; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

void setup() {
  
  //Initialize I2C communcations
  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceiveEvent);
  Wire.onRequest(onRequestEvent);
  Serial.begin(115200);
  Serial.println("Ready to receive float motion commands.");

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

  Serial.begin(115200); // Initialize serial communications
  last_time_ms = millis(); // Start time recording
  start_time_ms = last_time_ms;

  // Define interrupt functions for encoders
  attachInterrupt(digitalPinToInterrupt(M1EncA), M1Enc_Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), M2Enc_Update, CHANGE);

}

void loop() {

  current_time = (float)(last_time_ms-start_time_ms)/1000; // Update current time

  /*
  // Set desired position at 5 seconds for testing
  if (current_time >= 5) {
    desired_pos_xy[0] = 6 * 0.3048;
    desired_pos_xy[1] = 6 * 0.3048;
    // desired_phi = 2*pi;
  }
  */
  
  if (received) {
    Serial.print("Received Distance (ft): ");
    Serial.print(received_distance, 2);
    Serial.print(" | Rotation (deg): ");
    Serial.println(received_rotation, 2);
    received = false;
  }

  /*
  // Find desired robot distance and angle
  x_error = desired_pos_xy[0] - robot_position[0];
  y_error = desired_pos_xy[1] - robot_position[1];

  // Calculate robot angle error
  desired_phi = atan2(y_error,x_error);
  act_phi_error = desired_phi - phi; // Actual error between current angle and desired angle

  // Calculate phi error based on minimum angle needed to turn and determine whether robot should drive forwards or backwards
  if (act_phi_error > (pi/2)) { 
    direction = -1;
    phi_error = act_phi_error - pi;
  } else if (act_phi_error >= (-pi/2)) {
    direction = 1;
    phi_error = act_phi_error;
  } else if (act_phi_error < (-pi/2)) {
    direction = -1;
    phi_error = act_phi_error + pi;
  }
  
  // Increment phi integral error
  phi_integral_error = constrain(phi_integral_error + phi_error*(sample_time/1000.0f),-1,1);

  // Calculate rho errors
  rho_error = sqrt( (x_error*x_error) + (y_error*y_error) ) * direction;
  rho_integral_error = constrain(rho_integral_error + rho_error*(sample_time/1000.0f),-1,1);
  
  // Calculate the desired robot velocity and angular velocity via 2 PI controllers
  desired_robot_vel = Kp_rho*rho_error + Ki_rho*rho_integral_error;
  desired_robot_omega = Kp_phi*phi_error + Ki_phi*phi_integral_error;

  // If error is small, disable motor control
  if ( abs(rho_error) < DISTANCE_TOLERANCE && abs(actual_vel[0]) < 0.1 ) {
    desired_robot_vel = 0;
    desired_robot_omega = 0;
  }
  */

  for(int i=0;i<2;i++) {
    // Update motor positions
    wheel_rad[i] = 2*pi*(float)Enc_Counter[i]/counts_per_rev;

    // Calculate motor velocity
    actual_vel[i] = (wheel_rad[i] - prev_rad[i]) / (sample_time/1000.0f);

    // Calculate applied voltage
    applied_voltage_rho[i] = constrain(Kp_vel[i]*( (desired_robot_vel / WHEEL_RADIUS) - actual_vel[i] ),-BATTERY_VOLTAGE,BATTERY_VOLTAGE);
    applied_voltage_phi[i] = Kp_vel[i]*(desired_robot_omega * (WHEEL_BASE/(2*WHEEL_RADIUS)));

    // For odometry
    delta_M[i] = (wheel_rad[i] - prev_rad[i]) * WHEEL_RADIUS;
    prev_rad[i] = wheel_rad[i];
  }

  // Calculate PWM signal
  desired_PWM[0] = (abs(applied_voltage_rho[0]-applied_voltage_phi[0]) / BATTERY_VOLTAGE) * 255; // Find desired PWM signal
  desired_PWM[1] = (abs(applied_voltage_rho[1]+applied_voltage_phi[1]) / BATTERY_VOLTAGE) * 255;
  PWM[0] += constrain(desired_PWM[0] - PWM[0],-10,10); // Limit change in PWM to prevent slipping
  PWM[1] += constrain(desired_PWM[1] - PWM[1],-10,10);
  PWM[0] = constrain(PWM[0],0,255); // Clamp PWM from 0 to 255
  PWM[1] = constrain(PWM[1],0,255);

  // Set motor driver sign pins
  if ( (applied_voltage_rho[0]-applied_voltage_phi[0]) > 0) { digitalWrite(M1Voltage_Sign, HIGH); }  
  else { digitalWrite(M1Voltage_Sign, LOW); }
  if ( (applied_voltage_rho[1]+applied_voltage_phi[1]) > 0) { digitalWrite(M2Voltage_Sign, LOW); }  
  else { digitalWrite(M2Voltage_Sign, HIGH); }

  // Write PWM signals to motors
  analogWrite(M1PWM, PWM[0]);
  analogWrite(M2PWM, PWM[1]);

  // Calculate robot movement
  float delta_phi = (delta_M[1] - delta_M[0]) / WHEEL_BASE;
  float delta_center = (delta_M[0] + delta_M[1]) / 2.0;

  if (abs(delta_phi) < 1e-6) {
    // Approx straight line
    robot_position[0] += delta_center * cos(phi);
    robot_position[1] += delta_center * sin(phi);
  } else {
    // Arc motion correction
    float R = delta_center / delta_phi; // instantaneous turn radius
    robot_position[0] += R * (sin(phi + delta_phi) - sin(phi));
    robot_position[1] -= R * (cos(phi + delta_phi) - cos(phi));
  }
  phi += delta_phi;

  // Normalize phi
  if (phi > pi) phi -= 2 * pi;
  if (phi < -pi) phi += 2 * pi;

  while (millis()<last_time_ms + sample_time) {}// Wait until desired time passes to go top of the loop
  last_time_ms = millis(); // Update time

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

void onReceiveEvent(int numBytes) {
  if (numBytes >= 9) {
    union { byte b[4]; float f; } dist, rot;
    if (Wire.available()) {
      command = Wire.read();
    }
    for (int i = 0; i < 4; i++) {
      if (Wire.available()) dist.b[i] = Wire.read();
    }
    for (int i = 0; i < 4; i++) {
      if (Wire.available()) rot.b[i] = Wire.read();
    }    
    received_distance = dist.f;
    received_rotation = rot.f;
    received = true;
  }

  switch (command) {
    case 0: 
      //spinning = true;
      rho_error = 0;
      phi_error = 0;
      desired_robot_vel = 0;              // no forward motion
      desired_robot_omega = 1.0;          // rad/s — adjust spin speed
      break;
    case 1:
      //spinning = false; 
      rho_error = received_distance;
      phi_error = received_rotation;
      desired_robot_vel = Kp_rho*rho_error + Ki_rho*rho_integral_error;
      desired_robot_omega = Kp_phi*phi_error + Ki_phi*phi_integral_error;
      break;
    case 2:
      //90 degree left turn
      rho_error = 0;
      phi_error = pi/2;
      desired_robot_vel = 0;              // no forward motion
      desired_robot_omega = Kp_phi*phi_error + Ki_phi*phi_integral_error;
      break;
    case 3:
      //90 degree right turn
      rho_error = 0;
      phi_error = -pi/2;
      desired_robot_vel = 0;              // no forward motion
      desired_robot_omega = Kp_phi*phi_error;
      break;
  }
}

void onRequestEvent() {
  union { byte b[4]; float f; } dist, rot;
  dist.f = received_distance;
  rot.f = received_rotation;
  for (int i = 0; i < 4; i++) Wire.write(dist.b[i]);
  for (int i = 0; i < 4; i++) Wire.write(rot.b[i]);
}