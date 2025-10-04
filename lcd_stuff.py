from time import sleep
from smbus2 import SMBus, i2c_msg
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import time


def LCD(north, west):
    # Make Bytes
    north = int(bool(north)) & 0xFF
    west = int(bool(west)) & 0xFF
    #initialize LCD and I2C
    i2c_lcd = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, 16,2)
    lcd.color = (100, 0, 0)
    ARD = 0x08

    #Loop to send data
    with SMBus(1) as i2c:
        print("here")
        for i in range(20):
            try:
                send = [north, west]
                msg = i2c_msg.write(ARD, send)
                i2c.i2c_rdwr(msg)
                print("tried")
                sleep(.1)
                reply = i2c_msg.read(ARD, 2)
                i2c.i2c_rdwr(reply)
                check = list(reply)
                if len(check) == 2:
                    lcd.clear()
                    print(check[0])
                    print(check[1])
                    lcd.message = str(f'Pos: {check[0]} {check[1]}')
                    return
            except (IOError, OSError):
                print("Could not write data to Aruduino")
                sleep(0.1)
        lcd.clear()
        lcd.message = str('Bad Response')
        return