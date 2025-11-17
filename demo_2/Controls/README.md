# Seed Lab mini Project Controls code

This folder provides all the code for the controls portion of Demo 2. 

# Implementation Files
SteeringControl.ino: Arduino file to provide motor control to the robot using camera data from the PI

# Helper Files
helper_files folder contains unused/reference code to isolate remote start code from the rest of the arduino sketch

remoteStart.cpp: I2C slave code. Pairs with remoteStart.h to break up remote start arduino code into smaller chunks, not used in current implemenation.

remoteStart.h: Headder file for remoteStart.cpp. Not used in current implemenation.

remoteStart.ino: Remote start test file, use to test I2C without motor control. Acts as reference code copied to SteeringControl.ino