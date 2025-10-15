#Threaded LCD function
# Primary developer: Jason Huang
# 10/06/2025
# Description: This function is threaded in main and when it is called it takes its inputs,
# sends the position commands to the Arduino 
# reads in what was sent to the arduino and updates the lcd screen

from time import sleep
from smbus2 import SMBus, i2c_msg
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import time

#Function defined
def LCD(angle, lcd):
    lcd.clear()
    lcd.message = str(f'Angle is {angle}')
    return