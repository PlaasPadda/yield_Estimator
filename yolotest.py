#!/home/plaaspadda/skripsie/bin/python3
from ultralytics import YOLO

model = YOLO("yolo11m.pt")

results = model.track("/home/plaaspadda/skripsie",persist=True,show=True,tracker="bytetrack.yaml")


