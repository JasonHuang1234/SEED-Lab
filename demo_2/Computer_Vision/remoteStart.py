# Raspberry Pi Remote Start
# Primary developer: Kiera Crawford
# 10/15/2025
# Description: Transmits target positions to Arduino using I2C until user quits.
# The Arduino defaults to continuous turning until commanded otherwise.

from time import sleep
from smbus2 import SMBus, i2c_msg
import struct

ARD_ADDR = 0x08  # Arduino I2C address

# Command byte meanings (must match Arduino)
COMMANDS = {
    "turn": 0x00,
    "stop": 0x01,
    "target": 0x02,
    "left": 0x03,
    "right": 0x04
}

def send_target(distance, angle, command_name):
    """Send command + float distance + float angle over I2C"""
    if command_name not in COMMANDS:
        print(f"Invalid command '{command_name}'. Valid options: {list(COMMANDS.keys())}")
        return

    command = COMMANDS[command_name]

    # Pack floats as little-endian
    dist_bytes = struct.pack('<f', float(distance))
    ang_bytes = struct.pack('<f', float(angle))
    packet = bytes([command]) + dist_bytes + ang_bytes  # 9 bytes total

    with SMBus(1) as i2c:
        try:
            # Send packet
            msg = i2c_msg.write(ARD_ADDR, packet)
            i2c.i2c_rdwr(msg)

            # Arduino confirmation
            reply = i2c_msg.read(ARD_ADDR, 8)
            i2c.i2c_rdwr(reply)
            check = list(reply)

            dist_reply = struct.unpack('<f', bytes(check[0:4]))[0]
            ang_reply = struct.unpack('<f', bytes(check[4:8]))[0]

            print(f"Arduino confirmed: {dist_reply:.2f} m, {ang_reply:.2f}°")

        except OSError:
            print("I2C communication error — ensure Arduino is powered and connected.")


def main():
    print("Remote Start Interface (Ctrl+C to quit)")
    print("Commands: turn | stop | target | left | right")
    print("For 'target', input: <distance> <angle>")

    while True:
        try:
            user_in = input("Enter command: ").strip().lower()

            if user_in == "quit":
                print("Exiting...")
                break

            elif user_in == "turn":
                send_target(0, 0, "turn")

            elif user_in == "stop":
                send_target(0, 0, "stop")

            elif user_in in ["left", "right"]:
                send_target(0, 0, user_in)

            elif user_in.startswith("target"):
                parts = user_in.split()
                if len(parts) != 3:
                    print("Usage: target <distance> <angle>")
                    continue
                dist = float(parts[1])
                ang = float(parts[2])
                send_target(dist, ang, "target")

            else:
                print("Invalid command. Try again.")

            sleep(0.5)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except ValueError:
            print("Invalid numeric input.")

if __name__ == "__main__":
    main()
