from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

TARGET_DIST = 65     # in inches
TARGET_ANG = -45.25   # in degrees

# Pack floats into bytes (little-endian)
dist_bytes = struct.pack('<f', TARGET_DIST)
ang_bytes = struct.pack('<f', TARGET_ANG)
send = list(dist_bytes + ang_bytes)

ARD = 0x08  # Arduino I2C address

with SMBus(1) as i2c:
    for i in range(20):
        try:
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

            print(f"Arduino confirmed: {dist_reply:.2f} ft, {ang_reply:.2f}°")
            break

        except (IOError, OSError):
            print("I2C communication error.")
            sleep(0.1)