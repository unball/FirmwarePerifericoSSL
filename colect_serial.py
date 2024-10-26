import os
import serial
import json
from pathlib import Path
from collections import defaultdict

# IMPORTANT
# pip install pyserial

def save_json(data, path):
    with open(path, "w") as f:
        return json.dump(data, f, indent=4)

def main(): 
    save_logs_dir = Path("logs")
    os.makedirs(save_logs_dir, exist_ok=True)

    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
    ser.flushInput()
    buffer = []

    size = 0
    while True:
        try:
            ser_bytes = ser.readline()
            decoded_bytes = ser_bytes.decode()
            print(decoded_bytes)
            buffer.append(decoded_bytes)
            print(size)
            size += 1

        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            break

    ser.close()
    save_path = save_logs_dir / "buffer.json"
    save_json(buffer, save_path)
    # for key, val in buffer.items():
    #     save_path = save_logs_dir / f"{key}.json"
    #     save_json(val, save_path)

if __name__ == "__main__":
    main()