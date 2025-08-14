#!/home/plaaspadda/skripsie/bin/python3
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

steering = -12.5  # signed float
power = 200       # unsigned short (2 bytes)

# 'fH' = float (4 bytes) + unsigned short (2 bytes), little-endian
packet = struct.pack('<fH', steering, power)

ser.write(packet)

echo_line = ser.readline().decode(errors='ignore').strip()
if echo_line:
    print(f"Echo from ESP32: {echo_line}")

echo_line = ser.readline().decode(errors='ignore').strip()
if echo_line:
    print(f"Echo from ESP32: {echo_line}")

ser.close()
