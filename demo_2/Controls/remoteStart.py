# Raspberry Pi Remote start
# Primary developer: Kiera Crawford
# 10/15/2025
# Description: Transmits target positions to Arduino using I2C until user quits

from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

# May


# I am defining input as {dist angle}, than I will be defining command as a single byte input, 0x00 is turn, 0x01 stop, 0x02 is use inputs, 0x03 is left, 0x04 is right
def distandangle(input, command)
    ARD = 0x08  # Arduino I2C address
    with SMBus(1) as i2c:
        while True:
            try:
                parts = user_input.split()
                if len(parts) != 2:
                    print("Please enter exactly two values: distance and angle.")
                    continue

                target_dist = float(parts[0])
                target_ang = float(parts[1])
                cmd = command

                # Pack floats into bytes (little-endian)
                dist_bytes = struct.pack('<f', target_dist)
                ang_bytes = struct.pack('<f', target_ang)
                send = bytes([cmd]) + dist_bytes + ang_bytes

                # Send 8 bytes: 4 for distance, 4 for angle
                msg = i2c_msg.write(ARD, send)
                i2c.i2c_rdwr(msg)
                sleep(1)

                # Read 8-byte reply
                reply = i2c_msg.read(ARD, 8)
                i2c.i2c_rdwr(reply)
                check = list(reply)

                # Unpack floats
                dist_reply = struct.unpack('<f', bytes(check[0:4]))[0]
                ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]

                print(f"Arduino confirmed: {dist_reply:.2f} in, {ang_reply:.2f}°")

            except ValueError:
                print("Invalid input. Please enter numeric values for distance and angle.")
            except (IOError, OSError):
                print("I2C communication error. Exiting...")
                break