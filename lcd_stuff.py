from time import sleep
from smbus2 import SMBus, i2c_msg
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import time


def LCD(north, west, lcd):
    print(north)
    print(west)
    # Make Bytes
    north = int(bool(north)) & 0xFF
    west = int(bool(west)) & 0xFF
    #initialize i2c
    ARD = 0x08
    #Loop to send data
    with SMBus(1) as i2c:
        for i in range(20):
            try:
                send = [north, west]
                msg = i2c_msg.write(ARD, send)
                i2c.i2c_rdwr(msg)
                sleep(.1)
                reply = i2c_msg.read(ARD, 2)
                i2c.i2c_rdwr(reply)
                check = list(reply)
                if len(check) == 2: 
                    print(f"lcd{check[0]}")
                    print(f"lcd{check[1]}")
                    lcd.message = str(f'Pos: {check[0]} {check[1]}')
                    sleep(0.15)
                    return
            except (IOError, OSError):
                print("Could not write data to Aruduino")
                sleep(0.1)
        lcd.clear()
        lcd.message = str('Bad Response')
        return
