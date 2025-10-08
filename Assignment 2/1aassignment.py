from smbus2 import SMBus
from time import sleep

ARD_ADDR = 8
i2c = SMBus(1)

while True:
    # User input section
    offset = int(input("Enter an offset (7 to quit): "))
    if offset == 7:
        break
    text = input("Enter a string to send: ")
    byte_data = [ord(c) for c in text]  # Convert string to ASCII values

    # Send the entire string as a block
    try:
        i2c.write_i2c_block_data(ARD_ADDR, offset, byte_data)
        print("String sent successfully.")
    except OSError as e:
        print(f"I2C transmission failed: {e}")