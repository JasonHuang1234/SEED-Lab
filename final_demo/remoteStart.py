# remoteStart.py

from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

ARD = 0x08

COMMANDS = {
    "turn":    0x00,
    "stop":    0x01,
    "control": 0x02,
    "left":    0x03,
    "right":   0x04,
}

def send_command(distance, angle, command_name):
    print("send called:", command_name, distance, angle)

    if command_name not in COMMANDS:
        print(f"Invalid command '{command_name}'. Valid commands: {list(COMMANDS.keys())}")
        return

    cmd = COMMANDS[command_name]

    dist_bytes = struct.pack('<f', float(distance))
    ang_bytes  = struct.pack('<f', float(angle))
    send = bytes([cmd]) + dist_bytes + ang_bytes  # 9 bytes

    try:
        with SMBus(1) as i2c:
            msg = i2c_msg.write(ARD, send)
            i2c.i2c_rdwr(msg)

            # optional single quick reply (only if Arduino always replies fast)
            # reply = i2c_msg.read(ARD, 8)
            # i2c.i2c_rdwr(reply)

    except OSError:
        print("I2C communication error. Check Arduino connection or power.")

    sleep(0.01)


# remoteStart.py (same file, below send_command)

def poll_turn_done():
    """
    Non-blocking-ish check for 'turn complete'.
    Returns True if Arduino reported ang_reply == 180.0, else False.
    """
    try:
        with SMBus(1) as i2c:
            reply = i2c_msg.read(ARD, 8)
            i2c.i2c_rdwr(reply)
            check = list(reply)

            if len(check) >= 8:
                ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]
                # you'll probably want some tolerance here, but let's keep it exact for now:
                if ang_reply == 180.0:
                    print("Arduino reported turn complete (180)")
                    return True
    except OSError:
        # no reply / timeout / bus error = not done (or comm issue)
        pass

    return False
