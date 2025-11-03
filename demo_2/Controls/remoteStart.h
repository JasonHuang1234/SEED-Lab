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