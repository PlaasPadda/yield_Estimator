#!/home/plaaspadda/skripsie/bin/python3
import cv2
from ultralytics import YOLO
import FreeSimpleGUI as sg
import io 
from PIL import Image
import threading
import math
import evdev
from evdev import ecodes
from select import select
import serial
import struct
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

#---------------------------------------CONSTANTS-------------------------------
# Define target object as apples
TARGETS         = [47]
INPUT_DEVIDER   = pow(2,15)
ACK             = struct.pack('<H', 1)
#--------------------------------------Classes----------------------------------
class Controller():
    def __init__(self):
        self.LJoyX = 0 
        self.RTrig = 0
        self.B_BTN = 0
        self.X_BTN = 0
        self.Y_BTN = 0

        self.steerStren = 0
        self.torque = 0
        self.steer = "Middle"
        self.algo = "Off"

    def readController(self, Device, Window):  
        for event in Device.read():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:
                    self.LJoyX = event.value/INPUT_DEVIDER
                elif event.code == ecodes.ABS_RZ:
                    self.RTrig = event.value/255
            elif event.type == ecodes.EV_KEY:
                if event.code == ecodes.BTN_NORTH:
                    self.X_BTN = event.value
                elif event.code == ecodes.BTN_WEST:
                    self.Y_BTN = event.value
                elif event.code == ecodes.BTN_EAST:
                    self.B_BTN = event.value

        if (self.LJoyX > 0):
            self.steer = "Right"
        elif (self.LJoyX  < 0):
            self.steer = "Left"
        elif (self.LJoyX  == 0):
            self.steer = "Middle"

        if (self.X_BTN>0):
            self.algo = "On"

        if (self.B_BTN>0):
            self.algo = "Off"

        self.steerStren = int(self.LJoyX*100)
        self.torque = int(self.RTrig*100)
        Window['algo'].update(self.algo)
        #Window['torque'].update(self.torque)
        Window['steer'].update(self.steer)
        #Window['steerStren'].update(self.steerStren)

        return self.Y_BTN

    def sendControl(self):
        packet = struct.pack('<fH', self.steerStren, self.torque) 
        ser.write(packet)

    def updateControl(self, Window, Strstren, Torq):
        self.steerStren = Strstren
        self.torque = Torq

        Window['steerStren'].update(self.steerStren)  # When debugging controller receive
        Window['torque'].update(self.torque)


class AppleCounter():
    def __init__(self):
        self.aCount = 0
        self.ID_list = []

    def detectObjects(self, yoloFrame, targets, Window):
        # print id's and classes
        print(yoloFrame[0].boxes.id)
        print(self.ID_list)
        print(yoloFrame[0].boxes.cls)

        # list detected objects if there are any   
        if not (yoloFrame[0].boxes.id == None):
            detections_cls = yoloFrame[0].boxes.cls.tolist()
            detections_ID = yoloFrame[0].boxes.id.tolist()

            # loop through objects and check for hit
            for i in range(0, len(detections_cls)):
                if (detections_cls[i] in targets) and (detections_ID[i] not in self.ID_list):
                    self.ID_list.append(detections_ID[i])
                    self.aCount += 1
                    Window['aCount'].update(self.aCount)

    def refreshIDList(self, yoloFrame):
        detections_ID = yoloFrame[0].boxes.id.tolist()
        self.ID_list = detections_ID.copy()


class Plotter():
    def __init__(self, Window):
        self.x_pos_arr = np.array([])
        self.y_pos_arr = np.array([])
        self.x_vel = 0 
        self.y_vel = 0 
        
        self.fig, self.ax = plt.subplots()

        self.canvas = Window['-CANVAS-'].TKCanvas
        
        self.figure_canvas_agg = FigureCanvasTkAgg(self.fig, self.canvas)
        self.figure_canvas_agg.draw()
        self.figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)

    def updateValues(self, X, Y, XV, YV):
        self.x_pos_arr = np.append(self.x_pos_arr, X)
        self.y_pos_arr = np.append(self.y_pos_arr, Y)
        self.x_vel = XV 
        self.y_vel = YV

    def updateCanvas(self):
        self.ax.cla() #Clear plot
        self.ax.scatter(self.x_pos_arr, self.y_pos_arr, color="cyan")
        self.ax.set_xlabel(f"X(m) [{self.x_vel}]") 
        self.ax.set_ylabel(f"Y(m) [{self.y_vel}]") 
        self.ax.set_xlim(-30,30)
        self.ax.set_ylim(-30,30)
        self.figure_canvas_agg.draw()
        #print(self.x_vel)
        #print(self.y_vel)
        #print(self.x_pos_arr)
        #print(self.y_pos_arr)

#------------------------------------FUNCTIONS---------------------------------
def updateVideo(window, yoloFrame):
    scan = yoloFrame[0].plot()
    scan_rgb = cv2.cvtColor(scan, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(scan_rgb)

    with io.BytesIO() as output:
        img_pil.save(output, format="PNG")
        png_bytes = output.getvalue()

    window['scan'].update(data=png_bytes)

def read_values(ser):
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        print(line)
        if (line):  # skip empty
            try:
                x_pos = float(line[0:6])
                y_pos = float(line[6:12])
                x_vel = float(line[12:18])
                y_vel = float(line[18:24])
                steerStren = int(line[24:28])
                torque = int(line[28:31])
                break  # only exit once we got it
            except ValueError:
                print(f"Skipped bad line: {line}")

    return x_pos, y_pos, x_vel, y_vel, steerStren, torque

#----------------------------------MAIN---------------------------------------
if __name__=='__main__':

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)

    con_Connected = False
    print('Searching for controller...')
    input_list = evdev.list_devices()

    if not input_list:
        print('No devices found.')
    else:
        print(f"Devices found at {input_list}")
        print('Connecting controller...')
        device = evdev.InputDevice(f'{input_list[0]}')
        con_Connected = True
        print("Using device:", device)

    cam = cv2.VideoCapture(0)
    model = YOLO("yolo11m.pt")
    
    # Get the default frame width and height
    FRAME_WIDTH = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    FRAME_HEIGHT = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    GRAPH_SIZE = (FRAME_WIDTH, FRAME_HEIGHT)

    #----- Set up GUI window -----
    colRover = [[sg.Text('Driving UI')],
                [sg.Text('Steer: '), sg.Text('middle', key='steer'), sg.Text(0, key='steerStren'), sg.Text('Algorithm: '), sg.Text('Off', key='algo'), sg.Text('Torque: '), sg.Text(0, key='torque')],
                [sg.Text('Count:'), sg.Text(0,key='aCount')]]

    colMap = [[sg.Push(),sg.Text('Coordinates: '),sg.Push()],
              [sg.Text('Longitude: '), sg.Text(0, key='coorLong'), sg.Push(), sg.Text('Latitude: '), sg.Text(0, key='coorLat')],
              [sg.Push(), sg.Text('Orientation: '), sg.Push()]]

    layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
              [sg.Image('/home/plaaspadda/Pictures/PythExecute.png', key='scan',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
              [sg.Canvas(size=GRAPH_SIZE, key='-CANVAS-'), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)
    #-------------------------------
    
    appleCounter = AppleCounter()
    controller = Controller()
    plotter = Plotter(window)
    framecount = 0
    
    while True:
        # Check if window is closed
        event, values = window.read(timeout=0)

        if (event == sg.WIN_CLOSED):
            break
    
        #Read from camera
        returned, frame = cam.read()

        # Get detection results from yolo and bytetrack
        results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml")

        # Detect if desired objects are within frame
        appleCounter.detectObjects(yoloFrame=results, targets=TARGETS, Window=window)

        framecount += 1 

        # After every 120 frames, refresh ID list
        if (framecount>120):
            if not (results[0].boxes.id == None):
                appleCounter.refreshIDList(yoloFrame=results)
                framecount = 0

        # Update video feed on GUI
        updateVideo(window=window, yoloFrame=results)
        
        # If controller has events ready to be read, read and store them
        if (con_Connected == True):
            #select returns list of devices ready to be read from (first return) 
            devicesReady, _, _ = select([device], [], [], 0)   #(Timeout set to 0)
            if device in devicesReady:
                Y_BTN = controller.readController(Device=device, Window=window)

                if (Y_BTN>0):
                    Y_BTN = 0
                    break

        
        controller.sendControl()

        x, y, xv, yv, strstren, torq = read_values(ser) 

        controller.updateControl(Window = window, Strstren=strstren, Torq=torq)
        plotter.updateValues(X=x, Y=y, XV=xv, YV=yv)

        updateIndicator = framecount % 10 # Use framecount to also time canvas update to update every 10 frames
        if (updateIndicator == 0):
            plotter.updateCanvas()

    # Release capture and writer objects
    window.close()
    cam.release()
    cv2.destroyAllWindows()
