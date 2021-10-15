import torch
import time
import cv2
import io
from PIL import Image

# Load custom trained model
model = torch.hub.load('ultralytics/yolov5','custom', path='model_training_runs/ball2/weights/best.pt') 

# Capture PIL image
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

cap.set((cv2.CAP_PROP_FRAME_WIDTH, #Insert camera width))
cap.set((cv2.CAP_PROP_FRAME_HEIGHT, #Insert camera height))

ret, frame = cap.read()

# Pass image through model to get results
results = model(frame, size = 416)
coords = results.pandas().xyxy[0]

cap.release()