#!/home/plaaspadda/skripsie/bin/python3
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt 

LOOP_LEN = 200 

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

plt.plot(instances, x_pos_arr, color="black", label="Filter Value")
plt.plot(instances, x_pos_arr-x_pos_p_arr, color="blue", label="Range of Possible Values")
plt.plot(instances, x_pos_arr+x_pos_p_arr, color="blue",)
plt.title('X Position Uncertainty')
plt.xlabel('Samples')
plt.ylabel('Position (m)')
plt.legend()
plt.show()

plt.plot(instances, y_pos_arr, color="black", label="Filter Value")
plt.plot(instances, y_pos_arr-y_pos_p_arr, color="blue", label="Range of Possible Values")
plt.plot(instances, y_pos_arr+y_pos_p_arr, color="blue")
plt.title('Y Position Uncertainty')
plt.xlabel('Samples')
plt.ylabel('Position (m)')
plt.legend()
plt.show()

plt.plot(instances, x_vel_arr, color="black", label="Filter Value")
plt.plot(instances, x_vel_arr-x_vel_p_arr, color="blue", label="Range of Possible Values")
plt.plot(instances, x_vel_arr+x_vel_p_arr, color="blue")
plt.title('X Velocity Uncertainty')
plt.xlabel('Samples')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.show()

plt.plot(instances, y_vel_arr, color="black", label="Filter Value")
plt.plot(instances, y_vel_arr-y_vel_p_arr, color="blue", label="range")
plt.plot(instances, y_vel_arr+y_vel_p_arr, color="blue") 
plt.title('Y Velocity Uncertainty')
plt.xlabel('Samples')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.show()

plt.plot(x_pos_arr, y_pos_arr, color="cyan", label="Route", linewidth=3)
plt.title('Route Plot')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.gca().set_aspect('equal')
plt.legend()
plt.show()
