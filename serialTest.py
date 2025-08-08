#!/home/plaaspadda/skripsie/bin/python3
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

ser.write(b'Hello World\n')

received_data_bytes = ser.readline()
received_data_str = received_data_bytes.decode('utf-8').strip()
print(received_data_str)

ser.close()
