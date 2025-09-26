from smbus2 import SMBus
from time import sleep
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#i2c = board.I2C()
#lcd_columns = 16
#lcd_rows = 2
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
#lcd.color = [100,0 , 0]
#time.sleep(1)


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
    print(f"{reply}")



#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
#lcd.color = [100,0 , 0]
#time.sleep(1)
#lcd.message = str(reply)







