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
from inputs import get_gamepad
import datetime

class Controller(object):
    Max_Trig_Val = math.pow(2, 8)
    Max_Joy_Val = math.pow(2, 15)

    def __init__(self):
        self.LJoyX = 0
        self.X_BTN = 0
        self.Y_BTN = 0
        self.B_BTN = 0
        self.RTrig = 0

         # Start the monitor in a background thread
        thread = threading.Thread(target=self._monitor_Controller, daemon=True)
        thread.start()

    def GiveValue(self):
        leftX = self.LJoyX
        btn_X = self.X_BTN
        btn_Y = self.Y_BTN
        btn_B = self.B_BTN
        rightTrig = self.RTrig
        return leftX, btn_X, btn_Y, btn_B, rightTrig

    def _monitor_Controller(self):
        event = (get_gamepad())[0]
        if (event.code == 'ABS_X'):
            self.LJoyX = event.state / Controller.Max_Joy_Val
        elif (event.code == 'BTN_WEST'):
            self.Y_BTN = event.state
        elif (event.code == 'BTN_NORTH'):
            self.X_BTN = event.state
        elif (event.code == 'BTN_EAST'):
            self.B_BTN = event.state
        elif (event.code == 'ABS_RZ'):
            self.RTrig = event.state / Controller.Max_Trig_Val

if __name__=='__main__':
    controller = Controller()

    cam = cv2.VideoCapture(0)
    model = YOLO("yolo11m.pt")

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

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

    loops = 0

    scan = '/home/plaaspadda/Pictures/PythExecute.png'

    timer = datetime.datetime.now()

    colRover = [[sg.Text('Driving UI')],
                [sg.Text('Steer: '), sg.Text(steer, key='steer'), sg.Text(steerStren, key='steerStren'), sg.Text('Algorithm: '), sg.Text(algo, key='algo'), sg.Text('Torque: '), sg.Text(torque, key='torque')],
                [sg.Text('Count:'), sg.Text(aCount,key='aCount')]]

    colMap = [[sg.Push(),sg.Text('Coordinates: '),sg.Push()],
              [sg.Text('Longitude: '), sg.Text(coorLong, key='coorLong'), sg.Push(), sg.Text('Latitude: '), sg.Text(coorLat, key='coorLat')],
              [sg.Push(), sg.Text('Orientation: '), sg.Push()],
              [sg.Button('Update')]]

    layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
              [sg.Image(scan , key='scan',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
              [sg.Image('/home/plaaspadda/Pictures/PythExecute.png', subsample=2), sg.Column(colMap)]]

    window = sg.Window('O-Count', layout, finalize=True)

    while True:
        
        ret, frame = cam.read()

        event, values = window.read(timeout=0.1)

        if (event == sg.WIN_CLOSED):
            break

        # Write the frame to the output file
        results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml")
        #scan = results[0].plot()
        #scan_rgb = cv2.cvtColor(scan, cv2.COLOR_BGR2RGB)
        #img_pil = Image.fromarray(scan_rgb)
        #with io.BytesIO() as output:
        #    img_pil.save(output, format="PNG")
        #    png_bytes = output.getvalue()
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

        window['aCount'].update(aCount)
        
        if (((datetime.datetime.now() - timer).microseconds)/1000 > 1):
            timer = datetime.datetime.now()
            controller._monitor_Controller()
            LX, BX, BY, BB, RT = controller.GiveValue()

            if (LX > 0):
                steer = "Right"
            elif (LX < 0):
                steer = "Left"
            elif (LX == 0):
                steer = "Middle"

            if (BX>0):
                algo = "On"

            if (BB>0):
                algo = "Off"

            steerStren = int(LX*100)
            torque = int(RT*100)
            window['algo'].update(algo)
            window['torque'].update(torque)
            window['steer'].update(steer)
            window['steerStren'].update(steerStren)

            if (BY>0):
                break
        
        #window['scan'].update(data=png_bytes)
        
    # Release the capture and writer objects
    window.close()
    cam.release()
    cv2.destroyAllWindows()

