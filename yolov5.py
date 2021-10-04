import torch
import time
import picamera
import io
from PIL import Image

# Load custom trained model
model = torch.hub.load('ultralytics/yolov5','custom', path='model_training_runs/ball2/weights/best.pt') 

# Capture PIL image
stream = io.BytesIO()
with picamera.PICamera() as camera:
    camera.start_preview()
    time.sleep(2)
    camera.capture(stream, format='jpeg')
stream.seek(0)
img = Image.open(stream)

# Pass image through model to get results
results = model(img, size = 416)
coords = results.pandas().xyxy[0]