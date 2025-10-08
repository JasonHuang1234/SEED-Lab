# SEED-Lab
Here is a brief description of each of the files for the Mini Project in this repository. 

# Implementation Files
main.py - opens camera and determines quadrent of aruco marker. passes position to lcd_stuff.py

lcd_stuff.py - takes in two bools (north, west), uses that to write position information to lcd screen and arduino.

Final_Arduino_Code - Recieves data from Pi and updates wheel positions on the robot accordingly.

# Pre-combined Files
something.ino - Sets up the device as an I2C follower at address 0x08, enabling two-way communication with a Raspberry Pi.

PI_Controller.ino - Implements closed-loop position and velocity control for two DC motors using encoder feedback and PWM voltage commands. It calculates motor states in real time, applies proportional-integral control, and logs data for MATLAB analysis. 

EncoderMAT2vars.ino - Performs real-time odometry by tracking wheel encoder counts, computing robot position and orientation, and communicating with a Raspberry Pi via I2C. It updates goal positions based on received commands and logs motion data for external analysis.

CombinedCode.ino - single file for Arduino that contains all contributers necessary robot code. Contains control system logic for position recieved from Pi and Matlab encoder correction (PID controls).
