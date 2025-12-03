# Raspberry Pi Remote Start
# Primary developer: Kiera Crawford
# 10/15/2025
# Description: Transmits target positions to Arduino using I2C until user quits.
# The Arduino defaults to continuous turning until commanded otherwise.

from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

# Arduino I2C address
ARD = 0x08

# Command byte meanings (must match Arduino)
COMMANDS = {
    "turn": 0x00,
    "stop": 0x01,
    "control": 0x02,
    "left": 0x03,
    "right": 0x04,
    "angle": 0x05
}

def send_command(distance, angle, command_name):
    print("send called")
    print(f"Command sent: '{command_name}'")
    """
    Sends a command and two float values (distance, angle) to the Arduino.
    Can be called from any external script (e.g. vision or control loop).
    """

    if command_name not in COMMANDS:
        print(f"Invalid command '{command_name}'. Valid commands: {list(COMMANDS.keys())}")
        return

    cmd = COMMANDS[command_name]

    # Pack floats into bytes (little-endian)
    dist_bytes = struct.pack('<f', float(distance))
    ang_bytes = struct.pack('<f', float(angle))
    send = bytes([cmd]) + dist_bytes + ang_bytes  # 9 bytes total

    with SMBus(1) as i2c:
        try:
            # Send command + floats
            msg = i2c_msg.write(ARD, send)
            i2c.i2c_rdwr(msg)

            # Read 8-byte reply (2 floats back from Arduino)
            reply = i2c_msg.read(ARD, 8)
            i2c.i2c_rdwr(reply)
            check = list(reply)

            msg = i2c_msg.write(ARD, send)
            i2c.i2c_rdwr(msg)

            reply = i2c_msg.read(ARD, 8)
            i2c.i2c_rdwr(reply)
            check = list(reply)

            dist_reply = struct.unpack('<f', bytes(check[0:4]))[0]
            ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]

            # Unpack floats
            dist_reply = struct.unpack('<f', bytes(check[0:4]))[0]
            ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]

            print(f"Arduino confirmed: {dist_reply:.2f} m, {ang_reply:.2f}°")

        except OSError:
            print("I2C communication error. Check Arduino connection or power.")

    # Small delay for I2C stability
    sleep(0.01)