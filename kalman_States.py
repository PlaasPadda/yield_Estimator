#!/home/plaaspadda/skripsie/bin/python3
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt 

LOOP_LEN = 100

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

x_pos_arr   = np.zeros(LOOP_LEN)
y_pos_arr   = np.zeros(LOOP_LEN)
x_vel_arr   = np.zeros(LOOP_LEN)
y_vel_arr   = np.zeros(LOOP_LEN)
x_pos_p_arr = np.zeros(LOOP_LEN)
y_pos_p_arr = np.zeros(LOOP_LEN)
x_vel_p_arr = np.zeros(LOOP_LEN)
y_vel_p_arr = np.zeros(LOOP_LEN)
instances   = np.arange(LOOP_LEN)

for i in range(LOOP_LEN):

    x_pos = ser.readline().decode(errors='ignore').strip()
    if x_pos:
        print(f"X: {x_pos}")
        x_pos_arr[i] = float(x_pos)

    y_pos = ser.readline().decode(errors='ignore').strip()
    if y_pos:
        print(f"Y: {y_pos}")
        y_pos_arr[i] = float(y_pos)

    x_vel = ser.readline().decode(errors='ignore').strip()
    if x_vel:
        print(f"vX: {x_vel}")
        x_vel_arr[i] = float(x_vel)

    y_vel = ser.readline().decode(errors='ignore').strip()
    if y_vel:
        print(f"vY: {y_vel}")
        y_vel_arr[i] = float(y_vel)

    x_pos_p = ser.readline().decode(errors='ignore').strip()
    if x_pos_p:
        print(f"Xp: {x_pos_p}")
        x_pos_p_arr[i] = float(x_pos_p)

    y_pos_p = ser.readline().decode(errors='ignore').strip()
    if y_pos_p:
        print(f"Yp: {y_pos_p}")
        y_pos_p_arr[i] = float(y_pos_p)

    x_vel_p = ser.readline().decode(errors='ignore').strip()
    if x_vel_p:
        print(f"vXp: {x_vel_p}")
        x_vel_p_arr[i] = float(x_vel_p)

    y_vel_p = ser.readline().decode(errors='ignore').strip()
    if y_vel_p:
        print(f"vYp: {y_vel_p}")
        y_vel_p_arr[i] = float(y_vel_p)

ser.close()

plt.subplot(2,2,1)
plt.plot(instances, x_pos_arr, color="black", label="position")
plt.plot(instances, x_pos_arr-x_pos_p_arr, color="blue", label="range")
plt.plot(instances, x_pos_arr+x_pos_p_arr, color="blue")
plt.title('X position')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()

plt.subplot(2,2,2)
plt.plot(instances, y_pos_arr, color="black", label="position")
plt.plot(instances, y_pos_arr-y_pos_p_arr, color="blue", label="range")
plt.plot(instances, y_pos_arr+y_pos_p_arr, color="blue")
plt.title('Y position')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()

plt.subplot(2,2,3)
plt.plot(instances, x_vel_arr, color="black", label="position")
plt.plot(instances, x_vel_arr-x_vel_p_arr, color="blue", label="range")
plt.plot(instances, x_vel_arr+x_vel_p_arr, color="blue")
plt.title('X Velocity')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()

plt.subplot(2,2,4)
plt.plot(instances, y_vel_arr, color="black", label="position")
plt.plot(instances, y_vel_arr-y_vel_p_arr, color="blue", label="range")
plt.plot(instances, y_vel_arr+y_vel_p_arr, color="blue") 
plt.title('Y Velocity')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()

plt.show()
