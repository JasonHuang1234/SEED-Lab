# Seed Lab mini Project CV code

This folder provides all the code for the computer vision portion of the Mini Project

# Implementation Files
calibrate_cap - Captures a set amount of images to be used for camera calibration
camera_calibrate.py -> Calibrates checkboard pattersn from the camera.
direction.py -> Fucntion that determines the green or red box next to the aruco marker and returns a color for the rover to turn
lcd_stuff.py - function that takes in a string and outputs it onto a lcd
main.py - Function that currently process camera data and remotes it into the arduino
newcameracalibrate.py - This function calibrates the camera using charuco markers
remoteStart.py - this function communicates with the arduino
trial1.py - This function reads in camera data and outputs a command to only let the robot move to within 1.5 ft of the marker
trial2.py - this function does the trial1 stuff but also turns based on the color of the arrow. 