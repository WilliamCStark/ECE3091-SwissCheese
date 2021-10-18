import torch
import time
import cv2
import io
from PIL import Image

# Load custom trained model
model = torch.hub.load('ultralytics/yolov5','custom', path='best.pt') 
#image = 'imagegg.jpg'

# Capture PIL image
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

ret, frame = cap.read()
im_cv = cv2.imwrite('imagegg.jpg',frame)
image = 'imagegg.jpg'
# Pass image through model to get results
model.conf = 0.2
results = model(image, size = 416)
coords = results.pandas().xyxy[0]

print(coords)

cap.release()
