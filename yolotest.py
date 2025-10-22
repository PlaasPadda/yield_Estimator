#!/home/plaaspadda/skripsie/bin/python3
from ultralytics import YOLO

model = YOLO("yolov8m_Orchard.pt")

results = model.track(source="/home/plaaspadda/skripsie/report/Software/TestImage.jpg",persist=True,show=True,tracker="bytetrack.yaml", show_labels=True, show_conf=True, save=True)



