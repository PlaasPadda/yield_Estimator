#!/home/plaaspadda/skripsie/bin/python3
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt 

LOOP_LEN = 300

def read_next_value(tag, i, arr):
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:  # skip empty
            try:
                arr[i] = float(line)
                print(f"{tag}: {line}")
                ser.write(ack)
                break  # only exit once we got it
            except ValueError:
                print(f"Skipped bad line: {line}")

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

ack = struct.pack('<H', 1)
ser.write(ack)

#for i in range(LOOP_LEN):
#
#    x_pos = ser.readline().decode(errors='ignore').strip()
#    if x_pos:
#        print(f"X: {x_pos}")
#        x_pos_arr[i] = float(x_pos)
#        ser.write(ack)
#
#    y_pos = ser.readline().decode(errors='ignore').strip()
#    if y_pos:
#        print(f"Y: {y_pos}")
#        y_pos_arr[i] = float(y_pos)
#        ser.write(ack)
#
#    x_vel = ser.readline().decode(errors='ignore').strip()
#    if x_vel:
#        print(f"vX: {x_vel}")
#        x_vel_arr[i] = float(x_vel)
#        ser.write(ack)
#
#    y_vel = ser.readline().decode(errors='ignore').strip()
#    if y_vel:
#        print(f"vY: {y_vel}")
#        y_vel_arr[i] = float(y_vel)
#        ser.write(ack)
#
#    x_pos_p = ser.readline().decode(errors='ignore').strip()
#    if x_pos_p:
#        print(f"Xp: {x_pos_p}")
#        x_pos_p_arr[i] = float(x_pos_p)
#        ser.write(ack)
#
#    y_pos_p = ser.readline().decode(errors='ignore').strip()
#    if y_pos_p:
#        print(f"Yp: {y_pos_p}")
#        y_pos_p_arr[i] = float(y_pos_p)
#        ser.write(ack)
#
#    x_vel_p = ser.readline().decode(errors='ignore').strip()
#    if x_vel_p:
#        print(f"vXp: {x_vel_p}")
#        x_vel_p_arr[i] = float(x_vel_p)
#        ser.write(ack)
#
#    y_vel_p = ser.readline().decode(errors='ignore').strip()
#    if y_vel_p:
#        print(f"vYp: {y_vel_p}")
#        y_vel_p_arr[i] = float(y_vel_p)
#        ser.write(ack)

for i in range(LOOP_LEN):
    read_next_value("X", i, x_pos_arr)
    read_next_value("Y", i, y_pos_arr)
    read_next_value("vX", i, x_vel_arr)
    read_next_value("vY", i, y_vel_arr)
    read_next_value("Xp", i, x_pos_p_arr)
    read_next_value("Yp", i, y_pos_p_arr)
    read_next_value("vXp", i, x_vel_p_arr)
    read_next_value("vYp", i, y_vel_p_arr)

ser.close()

plt.plot(instances, x_pos_arr, color="black", label="position")
plt.plot(instances, x_pos_arr-x_pos_p_arr, color="blue", label="top range")
plt.plot(instances, x_pos_arr+x_pos_p_arr, color="cyan", label="bottom range")
plt.title('X position')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()
plt.show()

plt.plot(instances, y_pos_arr, color="black", label="position")
plt.plot(instances, y_pos_arr-y_pos_p_arr, color="blue", label="range")
plt.plot(instances, y_pos_arr+y_pos_p_arr, color="blue")
plt.title('Y position')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()
plt.show()

plt.plot(instances, x_vel_arr, color="black", label="position")
plt.plot(instances, x_vel_arr-x_vel_p_arr, color="blue", label="range")
plt.plot(instances, x_vel_arr+x_vel_p_arr, color="blue")
plt.title('X Velocity')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()
plt.show()

plt.plot(instances, y_vel_arr, color="black", label="position")
plt.plot(instances, y_vel_arr-y_vel_p_arr, color="blue", label="range")
plt.plot(instances, y_vel_arr+y_vel_p_arr, color="blue") 
plt.title('Y Velocity')
plt.xlabel('samples')
plt.ylabel('magnitude')
plt.legend()
plt.show()

plt.scatter(x_pos_arr, y_pos_arr, color="red", label="Route")
plt.title('Route Plot')
plt.xlabel('X position')
plt.ylabel('Y position')
plt.legend()
plt.show()
