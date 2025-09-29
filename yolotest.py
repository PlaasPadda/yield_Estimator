#!/home/plaaspadda/skripsie/bin/python3
from ultralytics import YOLO

model = YOLO("yolo11m.pt")

results = model.track(source="/home/plaaspadda/Videos/OrchardTestVideo(10secclips).mp4",persist=True,show=True,tracker="bytetrack.yaml", conf=0.45, show_labels=True, show_conf=False, save=True)



