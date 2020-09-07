import chessboardFunctions as cf
import cv2
import os
import matplotlib.pyplot as plt   
import numpy as np
import pickle
from pcClass import RPIDualCam
from PIL import Image
import copy
import time
import queue
import threading
from triggerFunction import checkTrigger


# %% Parameters
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 0.05)
winSize = (10,10)
#Output parameters
pps = 64
#Number of points to check
numPts = 20
angleThreshHorizontal = 10
angleThreshVertical = angleThreshHorizontal

outMargins = [17,17,18,17]

#Debug
doDebug = False
showCalibration = False
calibrationFolder = 'calibration'

#Corner detection parameters
minDistance = 25
qualityLevel = 0.01





#%% Triggering parameters
idleCounter = 0
captured = False

    
    
#%% Load images
names = ['c1', 'c2']




imgFolder = os.path.normpath(r"triggeredImages")
fusedFolder = 'fused'
c1Folder = 'c1'
c2Folder = 'c2'


#%% Prepare calibration
middlePoints = [0]*len(names)
tforms = [0]*len(names)
outputGrids = [0]*len(names)
validSquareNums = [0]*len(names)
squareNums = [0]*len(names)
corners = [0]*len(names)
squareLists = [0]*len(names)
middleSquares = [0]*len(names)



    
#%% CALIBRATION

fuseRatio = 2.05
try:
    camSetup = RPIDualCam('background')
    camSetup.toggleCapture()
    time.sleep(2)
    frame_c1 = cv2.cvtColor(camSetup.queues[0].get(), cv2.COLOR_RGB2BGR)
    frame_c2 = cv2.cvtColor(camSetup.queues[1].get(), cv2.COLOR_RGB2BGR)
    images = [frame_c1, frame_c2]
    
    
    for i in range(len(images)):
        with open(os.path.join(calibrationFolder, names[i] + '.pickle'), "rb") as f:
            objects = pickle.load(f)
            tforms[i] = objects[0]
            outputGrids[i] = objects[1]
            validSquareNums[i] = objects[2]
            squareNums[i] = objects[3]
            corners[i] = objects[4]
            squareLists[i] = objects[5]
            middleSquares[i] = objects[6]
#%% RECTIFICATION
    print("Start")

    #Start view thread
    k=0
    while True: #Images are now available from the queue
        #middleSquaresCopy = copy.deepcopy(middleSquares)
        
        start = time.time()
        frame_c1 = camSetup.queues[0].get()
        frame_c2 = camSetup.queues[1].get()
        images = [frame_c1, frame_c2]
        end = time.time()
        elapsed = end - start
        print("Time margin before slowing down is {:.2f} seconds".format(elapsed))
        
        
        
        
        #Rectification and fusion
        start = time.time()
        rectifiedImages = [0]*len(images)
        for j in range(len(images)):
            rectifiedImages[j] = cf.transformImage(tforms[j], images[j], outputGrids[j], pps, validSquareNums[j], squareNums[j])
        fused = cf.fuseImages(rectifiedImages, middleSquares, outputGrids, pps, outMargins, fuseRatio, debug=False)
        fused = cv2.cvtColor(fused, cv2.COLOR_RGB2BGR)
        
        
        #Save images
        imgName = str(int(time.time())) + '.png'
        cv2.imwrite(os.path.join(imgFolder, fusedFolder,imgName),fused)
        cv2.imwrite(os.path.join(imgFolder, names[0],imgName),rectifiedImages[0])
        cv2.imwrite(os.path.join(imgFolder, names[1],imgName),rectifiedImages[1])
            
            
            
        #%%Show result
        fused = cv2.resize(fused,(int(fused.shape[0]*0.5), int(fused.shape[1]*0.5)))
        cv2.imshow('FusedImg', fused)
        cv2.waitKey(1)
        
        
        end = time.time()
        elapsed = end - start
        print("Processing time is {:.2f} seconds".format(elapsed))
        k+=1
finally:
    camSetup.shutDown()
    cv2.destroyAllWindows()





        














