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
def LCD(north, west, lcd):
    # Make Bytes
    north = int(bool(north)) & 0xFF
    west = int(bool(west)) & 0xFF
    #initialize i2c
    ARD = 0x08
    #Loop to send data
    with SMBus(1) as i2c:
        for i in range(20):
            try:
                #cvts to msg, with SMBUS
                send = [north, west]
                msg = i2c_msg.write(ARD, send)
                #sends msg
                i2c.i2c_rdwr(msg)
                sleep(1)
                #takes in reply from arduino
                reply = i2c_msg.read(ARD, 2)
                # converts reply to python list
                i2c.i2c_rdwr(reply)
                check = list(reply)
                # convert to integers
                check[0] = int(check[0])
                check[1] = int(check[1])
                #Check for valid inputs
                if check[0] == 0 or check[0] == 1 or check[1] == 0 or check[1] == 1: 
                    print(f"lcd{check[0]}")
                    print(f"lcd{check[1]}")
                    lcd.clear()
                    sleep(0.15)
                    lcd.message = str(f'Pos: {int(check[0])} {int(check[1])}')
                    return
                #Check for valid inputs
                else:
                    break
            except (IOError, OSError):
                print("Could not write data to Aruduino")
                sleep(0.1)
        #prints error
        lcd.clear()
        lcd.message = str('Bad Response')
        return