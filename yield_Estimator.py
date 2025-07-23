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

#---------------------------------------CONSTANTS-------------------------------
# Define target object as apples
TARGETS = [47]
INPUT_DEVIDER = pow(2,15)

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
        Window['torque'].update(self.torque)
        Window['steer'].update(self.steer)
        Window['steerStren'].update(self.steerStren)

        return self.Y_BTN

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
        
#------------------------------------FUNCTIONS---------------------------------
def updateVideo(window, yoloFrame):
    scan = yoloFrame[0].plot()
    scan_rgb = cv2.cvtColor(scan, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(scan_rgb)

    with io.BytesIO() as output:
        img_pil.save(output, format="PNG")
        png_bytes = output.getvalue()

    window['scan'].update(data=png_bytes)

#----------------------------------MAIN---------------------------------------
if __name__=='__main__':


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
              [sg.Graph(canvas_size=GRAPH_SIZE, graph_bottom_left=(0,0), graph_top_right=GRAPH_SIZE, background_color='lightgreen', key='-GRAPH-'), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)
    #-------------------------------
    
    appleCounter = AppleCounter()
    controller = Controller()
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

    # Release capture and writer objects
    window.close()
    cam.release()
    cv2.destroyAllWindows()
