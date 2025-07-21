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

#--------------------------------------GLOBAlS----------------------------------
ID_list = []
aCount = 0

#------------------------------------FUNCTIONS---------------------------------
def detectObjects(yoloFrame, targets):
    # print id's and classes
    print(yoloFrame[0].boxes.id)
    print(yoloFrame[0].boxes.cls)

    # list detected objects
    detections_cls = yoloFrame[0].boxes.cls.tolist()
    detections_ID = yoloFrame[0].boxes.id.tolist()

    # loop through objects and check for hit
    for i in range(0, len(detections_cls)):
        if (detections_cls[i] in targets) and (detections_ID[i] not in ID_list):
            ID_list.append(detections_ID[i])

def updateVideo(window, yoloFrame):
    scan = yoloFrame[0].plot()
    scan_rgb = cv2.cvtColor(scan, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(scan_rgb)

    with io.BytesIO() as output:
        img_pil.save(output, format="PNG")
        png_bytes = output.getvalue()

    window['scan'].update(data=png_bytes)

if __name__=='__main__':

    print('Searching for controller...')
    input_list = evdev.list_devices()

    if not input_list:
        print('No devices found.')
    else:
        print(f"Devices found at {input_list}")
        print('Connecting controller...')
        device = evdev.InputDevice(f'{input_list[0]}')
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
                [sg.Text('Count:'), sg.Text(aCount,key='aCount')]]

    colMap = [[sg.Push(),sg.Text('Coordinates: '),sg.Push()],
              [sg.Text('Longitude: '), sg.Text(0, key='coorLong'), sg.Push(), sg.Text('Latitude: '), sg.Text(0, key='coorLat')],
              [sg.Push(), sg.Text('Orientation: '), sg.Push()]]

    layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
              [sg.Image('/home/plaaspadda/Pictures/PythExecute.png', key='scan',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
              [sg.Graph(canvas_size=GRAPH_SIZE, graph_bottom_left=(0,0), graph_top_right=GRAPH_SIZE, background_color='lightgreen', key='-GRAPH-'), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)
    
    #----- Main Loop -----
    while True:
        # Check if window is closed
        event, values = window.read(timeout=0.01)

        if (event == sg.WIN_CLOSED):
            break
    
        #Read from camera
        ret, frame = cam.read()

        #Get results from yolo and bytetrack
        results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml")

        detectObjects(yoloFrame=results, targets=TARGETS)

        updateVideo(window=window, yoloFrame=results)
