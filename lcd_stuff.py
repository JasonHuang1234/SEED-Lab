from time import sleep
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def LCD(x):
    location = []
    ARD_ADDR = 0x08
    i2c = SMBus(1)
    reg = 0x00
    while(True):
        inputnum = int(input("Enter a Number between 0 and 100(Greater to quit): "))
        if inputnum > 100 or inputnum < 0:
            print("Quitting")
            break
        try:
            i2c.write_byte(ARD_ADDR,inputnum)
        except IOError:
            print("Could not write data to Aruduino")
        sleep(.1)
        reply = i2c.read_byte(ARD_ADDR)
    if reply == 1:
        lcd.clear()
        lcd.color =[100, 0, 0]
        time.sleep(1)
        lcd.message = print(f'Goal Position: {location[0]} {location[1]}')
    else:
        lcd.clear()
        lcd.color =[100, 0, 0]
        time.sleep(1)
        lcd.message = print(f'Bad Response')

    
















