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
from matplotlib.patches import Wedge, Rectangle
import random
import time

#---------------------------------------CONSTANTS-------------------------------
# Define target object as apples
TARGETS         = [1]
INPUT_DEVIDER   = pow(2,15)
ACK             = struct.pack('<H', 1)

TREEAMOUNTX = 10
TREEAMOUNTY = 10

TREEDISTANCE = 2 # y distance between trees in a row (in meters)
HALFTREEAREA = 0.5 # half the length of a tree block (in meters)
ROADWIDTH = 1 # width of space between tree blocks (in meters)
ROVERWIDTH = 0.6 # width of rover (in meters)

LIGHTGREEN_THRESH = 100
YELLOW_THRESH = 60
ORANGE_THRESH = 30
RED_THRESH = 10
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
        self.direction = 1 

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
            self.direction = 1 
            Window['direction'].update("Forward")

        if (self.B_BTN>0):
            self.direction = -1 
            Window['direction'].update("Reverse")

        self.steerStren = int(self.LJoyX*100)
        self.torque = int(self.RTrig*100*self.direction)
        #Window['torque'].update(self.torque)
        Window['steer'].update(self.steer)
        #Window['steerStren'].update(self.steerStren)

        return self.Y_BTN

    def sendControl(self):
        packet = struct.pack('<fh', self.steerStren, self.torque) 
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

        currentCount = 0

        # list detected objects if there are any   
        if not (yoloFrame[0].boxes.id == None):
            detections_cls = yoloFrame[0].boxes.cls.tolist()
            detections_ID = yoloFrame[0].boxes.id.tolist()

            # loop through objects and check for hit
            for i in range(0, len(detections_cls)):
                if (detections_cls[i] in targets) and (detections_ID[i] not in self.ID_list):
                    self.ID_list.append(detections_ID[i])
                    self.aCount += 1
                    currentCount += 1
            
            Window['aCount'].update(self.aCount)
        
        return currentCount

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

    def giveRoute(self):
        return self.x_pos_arr, self.y_pos_arr

class Boom():
    def __init__(self, TLx, TLy, BRx, BRy):
        self.topLeft = (TLx, TLy)
        self.bottomRight = (BRx, BRy)  

        center = 0.5 * (TLy - BRy)

        self.treeLeftPos = (TLx, BRy+center) 
        self.treeLeftCount = 0
        #self.treeLeftCount = random.randint(0, 150)


        self.treeRightpos = (BRx, BRy+center)
        self.treeRightCount = 0
        #self.treeRightCount = random.randint(0, 150)

    def inArea(self, RoverX, RoverY):
        if (self.topLeft[0] < RoverX) and (RoverX < self.bottomRight[0]):
            if (self.bottomRight[1] < RoverY) and (RoverY < self.topLeft[1]):
                return True
       
        print(self.topLeft[0])
        print(self.topLeft[1])
        print(self.bottomRight[0])
        print(self.bottomRight[1])
        return False

    def updateCount(self, Count, Heading):
        # Under assumption that camera points right
        if (90 < Heading) and (Heading < 270):    # ONTHOU OM ARDUINO SEND PYTHON RECEIVE TE VERANDER VIR HEADING
            self.treeLeftCount = self.treeLeftCount + Count  
            # ONTHOU OM APPLECOUNTER N RETURN COUNT IN TE SIT IN DETECTOBJECTS FUNCTION (maak n areaCount, UPDATE               aCount en return areacount)
        else:
            self.treeRightCount = self.treeRightCount + Count

    def givePlots(self):
        return self.treeLeftPos, self.treeRightpos

    def returnColours(self):
        if (self.treeLeftCount >= LIGHTGREEN_THRESH):
            leftColour = "forestgreen"

        elif ((self.treeLeftCount >= YELLOW_THRESH) and (self.treeLeftCount < LIGHTGREEN_THRESH)):
            leftColour = "lawngreen"

        elif ((self.treeLeftCount >= ORANGE_THRESH) and (self.treeLeftCount < YELLOW_THRESH)):
            leftColour = "yellow"

        elif ((self.treeLeftCount >= RED_THRESH) and (self.treeLeftCount < ORANGE_THRESH)):
            leftColour = "orange"

        elif ((self.treeLeftCount > 0) and (self.treeLeftCount < RED_THRESH)):
            leftColour = "red"

        elif (self.treeLeftCount == 0):
            leftColour = "darkseagreen"

        if (self.treeRightCount >= LIGHTGREEN_THRESH):
            rightColour = "forestgreen"

        elif ((self.treeRightCount >= YELLOW_THRESH) and (self.treeRightCount < LIGHTGREEN_THRESH)):
            rightColour = "lawngreen"

        elif ((self.treeRightCount >= ORANGE_THRESH) and (self.treeRightCount < YELLOW_THRESH)):
            rightColour = "yellow"

        elif ((self.treeRightCount >= RED_THRESH) and (self.treeRightCount < ORANGE_THRESH)):
            rightColour = "orange"

        elif ((self.treeRightCount > 0) and (self.treeRightCount < RED_THRESH)):
            rightColour = "red"

        elif (self.treeRightCount == 0):
            rightColour = "darkseagreen"

        return leftColour, rightColour
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
                torque = int(line[28:32])
                heading = int(line[32:35])
                break  # only exit once we got it
            except ValueError:
                print(f"Skipped bad line: {line}")

    return x_pos, y_pos, x_vel, y_vel, steerStren, torque, heading

#----------------------------------MAIN---------------------------------------
if __name__=='__main__':

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)

    tree_list = []
    topleftx = (-HALFTREEAREA - 0.5*ROVERWIDTH) - (ROADWIDTH + (2*HALFTREEAREA))
    toplefty = (TREEDISTANCE) - TREEDISTANCE
    bottomrightx = topleftx + (ROADWIDTH + (2*HALFTREEAREA))    
    bottomrighty = 0 - TREEDISTANCE

    topleftyBASE = toplefty
    bottomrightyBASE = bottomrighty

    treeBlock = 0 # in which tree block are we now

    for i in range(TREEAMOUNTX):
        topleftx = topleftx + (ROADWIDTH + (2*HALFTREEAREA))
        bottomrightx = bottomrightx + (ROADWIDTH + (2*HALFTREEAREA))
        for j in range(TREEAMOUNTY):
            toplefty = toplefty + TREEDISTANCE
            bottomrighty = bottomrighty + TREEDISTANCE 
            
            # add boom object to list of trees
            tree_list.append(Boom(TLx=topleftx, TLy=toplefty, BRx=bottomrightx, BRy=bottomrighty))

            if (j == TREEAMOUNTY-1):
                toplefty = topleftyBASE
                bottomrighty = bottomrightyBASE

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
    model = YOLO("yolo11s_appleset.pt")
    
    # Get the default frame width and height
    FRAME_WIDTH = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    FRAME_HEIGHT = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    GRAPH_SIZE = (FRAME_WIDTH, FRAME_HEIGHT)

    #----- Set up GUI window -----
    colRover = [[sg.Text('Driving UI')],
                [sg.Text('Steer: '), sg.Text('middle', key='steer'), sg.Text(0, key='steerStren'), sg.Text('Algorithm: '), sg.Text('Forward', key='direction'), sg.Text('Torque: '), sg.Text(0, key='torque')],
                [sg.Text('Count:'), sg.Text(0,key='aCount')]]

    colMap = [[sg.Push(),sg.Text('Rover Velocity: '),sg.Push()],
              [sg.Text('X Velocity (m/s): '), sg.Text(0, key='XVEL'), sg.Push(), sg.Text('Y Velocity (m/s): '), sg.Text(0, key='YVEL')],
              [sg.Push(), sg.Text('Orientation: '), sg.Text(0, key='HEADING'), sg.Push()]]

    layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
              [sg.Image('/home/plaaspadda/Pictures/Legend.png', key='scan',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
              [sg.Canvas(size=GRAPH_SIZE, key='-CANVAS-'), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)
    #-------------------------------
    
    appleCounter = AppleCounter()
    controller = Controller()
    plotter = Plotter(window)
    framecount = 0
    lastArea = ((-HALFTREEAREA - 0.5*ROVERWIDTH), 0) 

    #Start Timer
    start_time = time.time()
    loopCounter = 0
    
    while True:
        # Check if window is closed
        event, values = window.read(timeout=0)

        if (event == sg.WIN_CLOSED):
            break

        loopCounter += 1
    
        #Read from camera
        returned, frame = cam.read()

        # Get detection results from yolo and bytetrack
        results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml", show_labels=True, show_conf=False, save=False)

        #results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml", show_labels=True, show_conf=False, save=False, classes=[47])

        # Detect if desired objects are within frame
        count = appleCounter.detectObjects(yoloFrame=results, targets=TARGETS, Window=window)

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

        x, y, xv, yv, strstren, torq, head = read_values(ser) 
        
        Window['XVEL'].update(xv)
        Window['YVEL'].update(yv)
        Window['HEADING'].update(head)

        controller.updateControl(Window = window, Strstren=strstren, Torq=torq)
        plotter.updateValues(X=x, Y=y, XV=xv, YV=yv)

        if (x > (lastArea[0]+(ROADWIDTH + (2*HALFTREEAREA)))): # If moved one block right
            treeBlock = treeBlock + TREEAMOUNTY 
            lastArea = (lastArea[0]+(ROADWIDTH + (2*HALFTREEAREA)), lastArea[1])

        if ((y > (lastArea[1]+TREEDISTANCE)) and (y < (TREEAMOUNTY*TREEDISTANCE))): # If moved one block up
            treeBlock = treeBlock + 1
            lastArea = (lastArea[0], (lastArea[1]+TREEDISTANCE))

        if ((y < lastArea[1]) and (y > 0)): # If moved one block down
            treeBlock = treeBlock - 1
            lastArea = (lastArea[0], (lastArea[1]-TREEDISTANCE))

        tree_list[treeBlock].updateCount(Count=count, Heading=head)

        if not (tree_list[treeBlock].inArea(x,y)):
            print("Misalignment Detected: rover no longer in expected block")

        updateIndicator = framecount % 10 # Use framecount to also time canvas update to update every 10 frames
        if (updateIndicator == 0):
            plotter.updateCanvas()

    # Release capture and writer objects
    window.close()
    cam.release()
    cv2.destroyAllWindows()

    #Print FPS
    end_time = time.time()
    fps = loopCounter / (end_time - start_time)
    print("Frames Per Second: ", fps)

    # set up plot parameters 
    fig, ax = plt.subplots()
    radius = 0.8 * (HALFTREEAREA) 

    for i in range(TREEAMOUNTX*TREEAMOUNTY):
        treeLeftPos, treeRightpos = tree_list[i].givePlots()

        bottomLeft = (treeLeftPos[0]+HALFTREEAREA, treeLeftPos[1]-(0.5*TREEDISTANCE))
        road = Rectangle(bottomLeft, ROADWIDTH, TREEDISTANCE, facecolor="navajowhite", edgecolor="navajowhite")
        ax.add_patch(road)

        leftColour, rightColour = tree_list[i].returnColours()

        leftWedge = Wedge(treeLeftPos, radius, 270, 90, fc=leftColour, ec=leftColour)
        ax.add_patch(leftWedge)

        rightWedge = Wedge(treeRightpos, radius, 90, 270, fc=rightColour, ec=rightColour)
        ax.add_patch(rightWedge)

    #Plot route
    x_arr, y_arr = plotter.giveRoute() 
    ax.plot(x_arr, y_arr, color="cyan", linewidth=3)

    # Set axis limits so wedges are visible
    #ax.set_xlim(-2, 6.5)
    #ax.set_ylim(-1, 7)
    ax.set_aspect('equal')
    ax.set_facecolor("darkseagreen")
    ax.axis('auto')
    ax.set_autoscale_on(True)

    plot = plt.gcf()

    layout = [[sg.Push(),sg.Canvas(size=(1000,1000), key='-CANVAS-'), sg.Push(), sg.Image('/home/plaaspadda/Pictures/Legend.png')]]
    window = sg.Window('CanvasTest', layout, finalize=True, element_justification='center')
    canvas = window['-CANVAS-'].TKCanvas

    figure_canvas_agg = FigureCanvasTkAgg(plot, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    print('Done')

    while True:
        event,value = window.read(timeout=1)
        if event == sg.WIN_CLOSED:
            break

    window.close()
