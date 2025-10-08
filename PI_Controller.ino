#define Motor_Enable 4
#define M1Voltage_Sign 7
#define M2Voltage_Sign 8
#define M1PWM 9
#define M2PWM 10
#define M1EncA 2
#define M2EncA 3
#define M1EncB 5
#define M2EncB 6

/*
Measured Motor Parameters:
K1 = 1.5  sigma1 = 15.75
K2 = 1.5  sigma2 = 14.5
*/

// Define bool to use to print finished
bool finished = false;

const int pi = 3.1415926538;
const float BATTERY_VOLTAGE = 7.8;
float applied_voltage[2] = {0,0};
float PWM[2];

// Define gain, speed, and error variables
float Kp_vel[2] = {5,5};
float Kp_pos[2] = {20,20};
float Ki_pos[2] = {2.5,2.5};

float desired_pos[2] = {0,0};
float desired_vel[2];

float actual_pos[2];
float last_pos[2] = {0,0};
float actual_vel[2];

float pos_error[2];
float integral_error[2];
float vel_error[2];

// Define volatile counter variables for motor encoders
volatile long int Enc_Counter[2] = {0,0};

// Define time keeping variables
unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

void setup() {

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
  Serial.println("Ready!");

}

void loop() {

  // Update current_time
  current_time = (float)(last_time_ms-start_time_ms)/1000;

  for(int i=0;i<2;i++) {

    // Update motor positions
    actual_pos[i] = 2*pi*(float)Enc_Counter[i]/3200;

    // Calculate motor velocity
    actual_vel[i] = (actual_pos[i] - last_pos[i]) / ((float)desired_Ts_ms / 1000);

    // Calculate position and integral errors
    pos_error[i] = desired_pos[i] - actual_pos[i];
    integral_error[i] = integral_error[i] + pos_error[i]*((float)desired_Ts_ms /1000);

    // Calculate desired velocity and velocity error 
    desired_vel[i] = Kp_pos[i] * pos_error[i] + Ki_pos[i] * integral_error[i];
    vel_error[i] = desired_vel[i] - actual_vel[i];

    // Calculate applied voltage
    applied_voltage[i] = Kp_vel[i]*vel_error[i];

    // Calculate PWM signal 
    PWM[i] = constrain( (abs(applied_voltage[i]) / BATTERY_VOLTAGE) * 255, 0, 255);

  }

  // Set motor driver sign pins
  if (applied_voltage[0] > 0) { digitalWrite(M1Voltage_Sign, HIGH); }  
  else { digitalWrite(M1Voltage_Sign, LOW); }
  if (applied_voltage[1] > 0) { digitalWrite(M2Voltage_Sign, LOW); }  
  else { digitalWrite(M2Voltage_Sign, HIGH); }

  // Write PWM signals to motors
  analogWrite(M1PWM, PWM[0]);
  analogWrite(M2PWM, PWM[1]);

  // Change desired postion at 1 second
  if (current_time >= 1 && current_time < 5) {
    desired_pos[0] = pi;
    desired_pos[1] = pi;
  }

  // Record data for 3 seconds
  if (current_time <= 6) {

    // Print time, voltage, position, and velocity
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(constrain(applied_voltage[0],-7.5,7.5));
    Serial.print("\t");
    Serial.print(actual_pos[0]);
    Serial.print("\t");
    Serial.println(actual_vel[0]);

  } else if (current_time >= 3) {

    // Print finished for MATLAB
    if (!finished) {
      Serial.println("Finished");
    }
    finished = true;

  }

  // Update motor positions
  for(int i=0;i<2;i++) {
    last_pos[i] = actual_pos[i];
  }

  while (millis()<last_time_ms + desired_Ts_ms) {
  // Wait until desired time passes to go top of the loop
  }

  last_time_ms = millis();

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
