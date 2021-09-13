import numpy as np
import cv2
import matplotlib
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv

# with open("train_labels.csv", newline='') as csvfile:
#     reader = csv.DictReader(csvfile)
#     for row in reader:
#         print(row['filename'], row['width'], row['height'])

height = 480
width = 640
Xtrain = np.empty((2,height, width, 3)).astype(int)
img = cv2.imread('images/im_0001.png')
Xtrain[0,:,:,:] = img
plt.imshow(Xtrain[0,:,:,:])
plt.show()
print(img)