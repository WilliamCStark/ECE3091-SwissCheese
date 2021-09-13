import numpy as np
import cv2
import matplotlib
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv

#Set device to GPU_indx if GPU is avaliable
GPU_indx = 0
print(torch.cuda.get_device_name(0))
device = torch.device(GPU_indx if torch.cuda.is_available() else 'cpu')

# Generate some data

# N = 1000
# X = (100*np.random.rand(N,64,64,3)).astype(int) # block images

# Y = np.random.randint(5,59,size=(N,2)) # block positions

# for j in range(N):
#     X[j,Y[j,1]-5:Y[j,1]+5,Y[j,0]-5:Y[j,0]+5,0] = 250
#     X[j,Y[j,1]-5:Y[j,1]+5,Y[j,0]-5:Y[j,0]+5,1] = 50
#     X[j,Y[j,1]-5:Y[j,1]+5,Y[j,0]-5:Y[j,0]+5,2] = 50

# plt.figure(figsize=(15,5))
# for j in range(4):
#     plt.subplot(1,4,j+1)
#     plt.imshow(X[j,:,:,:],extent=[0,64,64,0])
#     plt.plot(Y[j,0],Y[j,1],'bo')
# plt.show()

# Xtrain = X[0:500,:,:,:]
# Ytrain = Y[0:500,:]

# Xtest = X[500:,:,:,:]
# Ytest = Y[500:,:]


# Import test and training data'
filenames = list() # a list of the filenames corresponding to each image
Ytrain= list() # a list of the position of the centre of each ball bearing
width = 0
height = 0
with open("train_labels.csv", newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    first = True
    for row in reader:
        if not first:
            filenames.append(row['filename'])
            xpos = (float(row['xmin']) + float(row['xmax']))/2
            ypos = (float(row['ymin']) + float(row['ymax']))/2
            Ytrain.append([xpos,ypos])
            width = int(row['width'])
            height = int(row['height'])
        else:
            first = False
Ytrain = np.array(Ytrain)
# Import image data
Xtrain = np.empty((len(filenames),height, width, 3)).astype(int)
for i in range(len(filenames)):
    img = cv2.imread('images/' + filenames[i])
    Xtrain[i,:,:,:] = img
print('pass')
# Print training images
plt.figure(figsize=(15,5))
for j in range(4):
    plt.subplot(1,4,j+1)
    plt.imshow(Xtrain[j,:,:,:])
    plt.plot(Ytrain[j,0],Ytrain[j,1],'bo')
plt.show()
# Testing data
filenames = list() # a list of the filenames corresponding to each image
width = 0
height = 0
with open("test_labels.csv", newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    first = True
    for row in reader:
        if not first:
            filenames.append(row['filename'])
            width = int(row['width'])
            height = int(row['height'])
        else:
            first = False
# Import image data
Xtest = np.empty((len(filenames),height, width, 3)).astype(int)
for i in range(len(filenames)):
    img = cv2.imread('images/' + filenames[i])
    Xtest[i,:,:,:] = img
print('pass')
# Print testing images
plt.figure(figsize=(15,5))
for j in range(4):
    plt.subplot(1,4,j+1)
    plt.imshow(Xtest[j,:,:,:])
plt.show()


# Define a simple CNN
class Flatten(nn.Module):
    def forward(self, input):
        return input.view(input.size(0), -1)

# The Detector class will be our detection model
class Detector(nn.Module):

    def __init__(self,output_dim,image_channels):
        super().__init__()
        
        self.encoder = nn.Sequential(
            nn.Conv2d(image_channels, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(3, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(3, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            Flatten(),
            nn.Linear(14400,128),
            nn.ReLU(),
            nn.Linear(128,output_dim),
            nn.Tanh()
        )

    def forward(self, x):
        
        positions = self.encoder(x)
        
        return positions

# Now we'll declare our network
network = Detector(output_dim=2,image_channels=3).to(device)

Nepochs = 100
Nbatch = 8

optimizer = torch.optim.Adam(network.parameters(),lr=1e-4)

losses = []
for j in range(Nepochs):
    
    # Shuffle our training data after each epoch
    idxs = np.arange(Xtrain.shape[0])
    np.random.shuffle(idxs)
    
    Xtrain = Xtrain[idxs,:,:,:]
    Ytrain = Ytrain[idxs,:]
    
    # Loop over training data in batches of images
    batch_losses = []
    for k in range(int(Xtrain.shape[0]/Nbatch)):
        
        # Scale images and positions to be between 0 and 1 and convert to tensors for pytorch
        Xbatch = torch.from_numpy(Xtrain[Nbatch*k:Nbatch*(k+1),:,:,:]).float().transpose(1,3)/255
        Ybatch = torch.from_numpy(Ytrain[Nbatch*k:Nbatch*(k+1),:]).float()/64-0.5
        Xbatch = Xbatch.to(device)
        Ybatch = Ybatch.to(device)

        # Predict positions using neural network
        Ypred = network(Xbatch)
        
        # Calulate the loss (error between predictions and true values)
        loss = torch.sum((Ypred-Ybatch)**2)
        
        # Zero existing gradients
        optimizer.zero_grad()
        
        # Adjust neural network weights and biases by taken a step in a direction that reduces the loss
        loss.backward()
        optimizer.step()
        
        batch_losses.append(loss.item())

    losses.append(np.mean(batch_losses))
    
    
    plt.clf()
    plt.plot(losses)
    plt.ylabel('Loss (mean squared error)')
    plt.xlabel('Epochs')
    plt.grid()

k = np.random.randint(Xtest.shape[0]/Nbatch)
test_ims = torch.from_numpy(Xtest[Nbatch*k:Nbatch*(k+1),:,:,:]).float().transpose(1,3)/255
test_ims = test_ims.to(device)
Ypred = network(test_ims).cpu().detach().numpy()
print(Ypred)

plt.figure(figsize=(15,5))
for j in range(Nbatch):
    plt.subplot(1,Nbatch,j+1)
    plt.imshow(Xtest[Nbatch*k+j,:,:,:])
    
    pos_pred = np.empty(np.shape(Ypred))
    pos_pred[:,0] = (Ypred[:,0]+0.5)*640 # Scale predictions to pixel coordinates
    pos_pred[:,1] = (Ypred[:,1]+0.5)*480 # Scale predictions to pixel coordinates
    
    # Show predicted marker positions in image
    plt.plot([pos_pred[j,0]],[pos_pred[j,1]],'bo',markersize=5)
plt.show()

im = Xtest[100,:,:,:].reshape(1,64,64,3)
torch_im = torch.from_numpy(im).transpose(1,3)/255.0 # Convert to torch tensor, reshape and normalise
torch_pos = network(torch_im.to(device)) # Feed through network

pos = (torch_pos.cpu().detach().numpy()+0.5)*64 #Convert back to numpy

print('Detected position: ',pos)