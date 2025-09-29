#!/home/plaaspadda/skripsie/bin/python3
import cv2
from ultralytics import YOLO
from yolox.tracker.byte_tracker import BYTETracker, STrack
from dataclasses import dataclass
@dataclass(frozen=True)
class BYTETrackerArgs:
    track_thresh: float = 0.25
    track_buffer: int = 30
    match_thresh: float = 0.8
    aspect_ratio_thresh: float = 3.0
    min_box_area: float = 1.0
    mot20: bool = False

cam = cv2.VideoCapture(0)
model = YOLO("yolo11m.pt")
byte_tracker = BYTETracker(BYTETrackerArgs())

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

# Define desired objects
targets = [47]

ID_list = []
count = 0
while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    results = model.track(frame, persist=True, show=False, tracker="bytetrack.yaml")
    out.write(results[0].plot())

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
    #cv2.imshow('Camera', results[0].plot())
    print(ID_list)
    print(len(ID_list))
    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
out.release()
cv2.destroyAllWindows()
