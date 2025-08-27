#!/home/plaaspadda/skripsie/bin/python3
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
count = 0

while True:
    count+=1

    x_pos = ser.readline().decode(errors='ignore').strip()
    if x_pos:
        print(f"X: {x_pos}")

    y_pos = ser.readline().decode(errors='ignore').strip()
    if y_pos:
        print(f"Y: {y_pos}")

    x_vel = ser.readline().decode(errors='ignore').strip()
    if x_vel:
        print(f"vX: {x_vel}")

    y_vel = ser.readline().decode(errors='ignore').strip()
    if y_vel:
        print(f"vY: {y_vel}")

    x_pos_p = ser.readline().decode(errors='ignore').strip()
    if x_pos_p:
        print(f"Xp: {x_pos_p}")

    y_pos_p = ser.readline().decode(errors='ignore').strip()
    if y_pos_p:
        print(f"Yp: {y_pos_p}")

    x_vel_p = ser.readline().decode(errors='ignore').strip()
    if x_vel_p:
        print(f"vXp: {x_vel_p}")

    y_vel_p = ser.readline().decode(errors='ignore').strip()
    if y_vel_p:
        print(f"vYp: {y_vel_p}")

    if (count == 10):
        break;

ser.close()

