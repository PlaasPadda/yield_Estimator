#!/home/plaaspadda/skripsie/bin/python3
import cv2
from ultralytics import YOLO
from yolox.tracker.byte_tracker import BYTETracker, STrack
from dataclasses import dataclass
import FreeSimpleGUI as sg
import io 
from PIL import Image
import threading
import math
import evdev
from evdev import ecodes
from select import select

if __name__=='__main__':

    print(evdev.list_devices())
    device = evdev.InputDevice('/dev/input/event21')
    print("Using device:", device)

    cam = cv2.VideoCapture(0)
    model = YOLO("yolo11m.pt")

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    GRAPH_SIZE = (frame_width, frame_height)

    # Define desired objects
    targets = [47]
    ID_list = []

    aCount = 0
    coorLong = 0
    coorLat = 0
    steerStren = 0
    torque = 0
    steer = "Middle"
    algo = "Off"
    
    LJoyX = 0
    RTrig = 0
    B_BTN = 0
    X_BTN = 0
    Y_BTN = 0
    devider = pow(2,15)
    loops = 0

    scan = '/home/plaaspadda/Pictures/PythExecute.png'

    colRover = [[sg.Text('Driving UI')],
                [sg.Text('Steer: '), sg.Text(steer, key='steer'), sg.Text(steerStren, key='steerStren'), sg.Text('Algorithm: '), sg.Text(algo, key='algo'), sg.Text('Torque: '), sg.Text(torque, key='torque')],
                [sg.Text('Count:'), sg.Text(aCount,key='aCount')]]

    colMap = [[sg.Push(),sg.Text('Coordinates: '),sg.Push()],
              [sg.Text('Longitude: '), sg.Text(coorLong, key='coorLong'), sg.Push(), sg.Text('Latitude: '), sg.Text(coorLat, key='coorLat')],
              [sg.Push(), sg.Text('Orientation: '), sg.Push()]]

    layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
              [sg.Image(scan , key='scan',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
              [sg.Graph(canvas_size=GRAPH_SIZE, graph_bottom_left=(0,0), graph_top_right=GRAPH_SIZE, background_color='lightgreen', key='-GRAPH-'), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)

    while True:
        
        ret, frame = cam.read()

        event, values = window.read(timeout=0.01)

        if (event == sg.WIN_CLOSED):
            break

        # Write the frame to the output file
        results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml")
        scan = results[0].plot()
        scan_rgb = cv2.cvtColor(scan, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(scan_rgb)
        with io.BytesIO() as output:
            img_pil.save(output, format="PNG")
            png_bytes = output.getvalue()
        # print id's and classes
        print(results[0].boxes.id)
        print(results[0].boxes.cls)

        # list detected objects
        detections_cls = results[0].boxes.cls.tolist()
        detections_ID = results[0].boxes.id.tolist()

        # loop through objects and check for hit
        for i in range(0, len(detections_cls)):
            if (detections_cls[i] in targets) and (detections_ID[i] not in ID_list):
                ID_list.append(detections_ID[i])

        # Display the captured frame
        print(ID_list)
        print(len(ID_list))
	
        aCount = len(ID_list)
        window['aCount'].update(aCount)
        
        r, _, _ = select([device], [], [], 0.01)
        if device in r:
            for event in device.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        LJoyX = event.value/devider
                    elif event.code == ecodes.ABS_RZ:
                        RTrig = event.value/255
                elif event.type == ecodes.EV_KEY:
                    if event.code == ecodes.BTN_NORTH:
                        X_BTN = event.value
                    elif event.code == ecodes.BTN_WEST:
                        Y_BTN = event.value
                    elif event.code == ecodes.BTN_EAST:
                        B_BTN = event.value

            if (LJoyX > 0):
                steer = "Right"
            elif (LJoyX  < 0):
                steer = "Left"
            elif (LJoyX  == 0):
                steer = "Middle"

            if (X_BTN>0):
                algo = "On"

            if (B_BTN>0):
                algo = "Off"

            steerStren = int(LJoyX*100)
            torque = int(RTrig*100)
            window['algo'].update(algo)
            window['torque'].update(torque)
            window['steer'].update(steer)
            window['steerStren'].update(steerStren)

            if (Y_BTN>0):
                break
        
        window['scan'].update(data=png_bytes)
        
    # Release the capture and writer objects
    window.close()
    cam.release()
    cv2.destroyAllWindows()

