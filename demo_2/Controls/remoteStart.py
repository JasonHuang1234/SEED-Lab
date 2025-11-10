# Raspberry Pi Remote Start
# Primary developer: Kiera Crawford
# 10/15/2025
# Description: Transmits target positions to Arduino using I2C until user quits.
# The Arduino defaults to continuous turning until commanded otherwise.

from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

def remote_start():
    ARD_ADDR = 0x08  # Arduino I2C address
    COMMANDS = {
        "turn": 0x00,    # Default state: keep turning until told otherwise
        "control": 0x01, # Pi-controlled motion (includes stop and target behavior)
        "left": 0x03,
        "right": 0x04
    }

    print("Remote Start Interface (Ctrl+C to quit)")
    print("Commands: turn | control | left | right")
    print("For 'control', input: <distance> <angle> (0,0 = stop)")

    while True:
        try:
            user_in = input("Enter command: ").strip().lower()
            if user_in == "quit":
                print("Exiting...")
                break

            # Determine command and parameters
            if user_in in ["turn", "left", "right"]:
                cmd = COMMANDS[user_in]
                dist, ang = 0.0, 0.0

            elif user_in.startswith("control"):
                parts = user_in.split()
                if len(parts) != 3:
                    print("Usage: control <distance> <angle>")
                    continue
                cmd = COMMANDS["control"]
                dist = float(parts[1])
                ang = float(parts[2])

            else:
                print("Invalid command. Try again.")
                continue

            # Build I2C packet
            dist_bytes = struct.pack('<f', dist)
            ang_bytes = struct.pack('<f', ang)
            packet = bytes([cmd]) + dist_bytes + ang_bytes  # 9 bytes total

            with SMBus(1) as i2c:
                try:
                    # Send packet
                    msg = i2c_msg.write(ARD_ADDR, packet)
                    i2c.i2c_rdwr(msg)

                    # Receive Arduino confirmation (8 bytes)
                    reply = i2c_msg.read(ARD_ADDR, 8)
                    i2c.i2c_rdwr(reply)
                    check = list(reply)

                    dist_reply = struct.unpack('<f', bytes(check[0:4]))[0]
                    ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]

                    print(f"Arduino confirmed: {dist_reply:.2f} m, {ang_reply:.2f}°")

                except OSError:
                    print("I2C communication error — ensure Arduino is powered and connected.")

            sleep(0.5)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except ValueError:
            print("Invalid numeric input.")

if __name__ == "__main__":
    remote_start()