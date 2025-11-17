// Remote Start header file
// Primary developer: Kiera Crawford
// Updated: 10/23/2025
// Description: Header file to reduce size of main sketch
// To use include in main sketch and

#ifndef REMOTE_START_H
#define REMOTE_START_H

#include <Wire.h>
#define MY_ADDR 0x08

// Extern should make it so that the main sketch can grab these values
extern float received_distance;
extern float received_rotation;
extern bool received;

// Function Declarations
void RemoteStart_setup();
void RemoteStart_loop();
void onReceiveEvent(int numBytes);
void onRequestEvent();

#endif