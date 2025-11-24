#Threaded LCD function
# Primary developer: Jason Huang
# 10/06/2025
# Description: This function takes in an input and prints it onto the LCD screen

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