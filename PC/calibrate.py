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
angleThreshHorizontal = 15
angleThreshVertical = angleThreshHorizontal

outMargins = [17,17,18,17]

#Debug
doDebug = True
showCalibration = False
calibrationFolder = 'calibration'
calibrate = False

#Corner detection parameters
minDistance = 25
qualityLevel = 0.01





#%% Triggering parameters
idleCounter = 0
captured = False

    
    
#%% Load images
names = ['c1', 'c2']



#%% Prepare calibration
middlePoints = [0]*len(names)
tforms = [0]*len(names)
outputGrids = [0]*len(names)
validSquareNums = [0]*len(names)
squareNums = [0]*len(names)
corners = [0]*len(names)
squareLists = [0]*len(names)
middleSquares = [0]*len(names)


fuseRatio = 2.1
    
#%% CALIBRATION


try:
    camSetup = RPIDualCam('calibrate')
    camSetup.toggleCapture()
    time.sleep(2)
    frame_c1 = cv2.cvtColor(camSetup.queues[0].get(), cv2.COLOR_RGB2BGR)
    frame_c2 = cv2.cvtColor(camSetup.queues[1].get(), cv2.COLOR_RGB2BGR)
    images = [frame_c1, frame_c2]

    for i in range(len(images)):
        middlePoints[i] = cf.findRedPoint(images[i], debug=True)
        tforms[i], outputGrids[i], validSquareNums[i], squareNums[i], corners[i], squareLists[i] = cf.calibrateCamera(images[i], middlePoints[i], minDistance, qualityLevel, criteria, winSize, pps, numPts, angleThreshHorizontal, angleThreshVertical, debug=doDebug)
        imin = cf.getMiddleSquareIndex(middlePoints[i], squareLists[i])
        middleSquares[i] = cf.getGridCorners(validSquareNums[i][imin], squareNums[i], pps)
        
        #Store result
        cv2.imwrite(os.path.join(calibrationFolder, names[i] + '.png'), images[i])
        with open(os.path.join(calibrationFolder, names[i] + '.pickle'), 'wb') as f:
            pickle.dump([tforms[i], outputGrids[i], validSquareNums[i], squareNums[i], corners[i], squareLists[i], middleSquares[i]], f)
    
        #End of calibration, continue normal operation
        calibrate = False
    else:
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

    images = [cv2.cvtColor(frame_c1, cv2.COLOR_BGR2RGB), cv2.cvtColor(frame_c2, cv2.COLOR_BGR2RGB)]
    rectifiedImages = [0]*len(images)   	
    for j in range(len(images)):
        rectifiedImages[j] = cf.transformImage(tforms[j], images[j], outputGrids[j], pps, validSquareNums[j], squareNums[j])
        plt.figure()
        plt.imshow(rectifiedImages[j])
    fused = cf.fuseImages(rectifiedImages, middleSquares, outputGrids, pps, outMargins, fuseRatio, debug=False)
    fused = cv2.cvtColor(fused, cv2.COLOR_RGB2BGR)
    
    #%% Show result
    plt.figure()
    plt.imshow(fused)
        
      
finally:
    camSetup.shutDown()
    cv2.destroyAllWindows()





        














