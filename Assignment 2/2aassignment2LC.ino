#define Motor_Enable 4
#define M1Voltage_Sign 7
#define M2Voltage_Sign 8
#define M1PWM 9
#define M2PWM 10
#define M1EncA 2
#define M2EncA 3
#define M1EncB 5
#define M2EncB 6

// Define bool to use to print finished
bool finished = false;
const int pi = 3.1415926538;
const float BATTERY_VOLTAGE = 7.8;
float applied_voltage_1 = 0;
float applied_voltage_2 = 0;

// Define gain, speed, and error variables
float Kp1 = 3;
float Kp2 = 3;
float desired_speed = (-4)*pi;
float error_1;
float error_2;

// Define volatile counter variables for motor encoders
volatile long int M1Enc_Counter = 0;
volatile long int M2Enc_Counter = 0;

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
 // Update motor positions
 float M1_rad = 2*pi*(float)M1Enc_Counter/3200;
 float M2_rad = 2*pi*(float)M2Enc_Counter/3200;
 // Motor velocity calculations
 static float lastM1_rad = M1_rad;
 static float lastM2_rad = M2_rad;
 float M1_omega = (M1_rad - lastM1_rad) / ( (float)desired_Ts_ms / 1000 );
 float M2_omega = (M2_rad - lastM2_rad) / ( (float)desired_Ts_ms / 1000 );
 // Calculate motor veolcity error
 error_1 = desired_speed - M1_omega;
 error_2 = desired_speed - M2_omega;
 // Apply voltage to motor between 1 and 3 seconds
 if (current_time >= 1 && current_time <= 3) {
 // Set voltage based on error
 applied_voltage_1 = Kp1 * error_1;
 applied_voltage_2 = Kp2 * error_2;
 // Set motor 1 driver sign pin
 if (applied_voltage_1 > 0) {
 digitalWrite(M1Voltage_Sign, HIGH);
 } else {
 digitalWrite(M1Voltage_Sign, LOW);
 }
 // Set motor 2 driver sign pin
 if (applied_voltage_2 > 0) {
 digitalWrite(M2Voltage_Sign, LOW);
 } else {
 digitalWrite(M2Voltage_Sign, HIGH);
 }
 // Print time, voltage, and velocity
 Serial.print(current_time);
 Serial.print("\t");
 Serial.print(applied_voltage_2);
 Serial.print("\t");
 Serial.println(M2_omega);
 // Write PWM signals to motor pins based on applied voltage
 unsigned int PWM_1 = constrain((abs(applied_voltage_1) / BATTERY_VOLTAGE) * 255, 0,
255);
 unsigned int PWM_2 = constrain((abs(applied_voltage_2) / BATTERY_VOLTAGE) * 255, 0,
255);
 analogWrite(M1PWM, PWM_1);
 analogWrite(M2PWM, PWM_2);
 } else if (current_time >= 3) {
 // Turn off mootrs
 analogWrite(M1PWM, 0);
 analogWrite(M2PWM, 0);
 // Print finished for MATLAB
 if (!finished) {
 Serial.println("Finished");
 }
 finished = true;
 }
 // Update motor positions
 lastM1_rad = M1_rad;
 lastM2_rad = M2_rad;
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
 if (A == B) { M1Enc_Counter += 2; }
 else { M1Enc_Counter -= 2; }
}

void M2Enc_Update() { // Interrupt function for motor 2's encoder
 // Read encoder pins
 int A = digitalRead(M2EncA);
 int B = digitalRead(M2EncB);
 // Increment count based on direction of rotation
 if (A == B) { M2Enc_Counter -= 2; }
 else { M2Enc_Counter += 2; }
}