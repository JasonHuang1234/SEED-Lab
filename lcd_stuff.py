from time import sleep
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import time

def LCD(north, west):
    i2c_lcd = board.I2C()
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    location = []
    ARD_ADDR = 0x08
    i2c = SMBus(1)
    while(True):
        inputnum = int(x)
        print("here")
        try:
            i2c.write_byte(ARD_ADDR,inputnum)
            print("tried")
        except IOError:
            print("Could not write data to Aruduino")
        sleep(.1)
        reply = i2c.read_byte(ARD_ADDR)
        if reply != None:
            break
    print(reply)
    if reply < 4 and  reply  >= 0:
        match x:
            case 0:
                location = (0, 0)
            case 1:
                location = (0, 1)
            case 2:
                location = (1, 1)
            case 3:
                location = (1, 0)
            case _:
                location = (-1, -1)
        lcd.clear()
        lcd.color =[100, 0, 0]
        print(location[0])
        print(location[1])
        sleep(1)
        lcd.message = str(f'Pos: {location[0]} {location[1]}')
        sleep(100)
    else:
        lcd.clear()
        lcd.color =[100, 0, 0]
        sleep(1)
        lcd.message = str(f'Bad Response')
LCD(1,0)
quit()















