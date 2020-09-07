import cv2
import numpy as np
import matplotlib.pyplot as plt



def checkTrigger(img, prevGray, idleCounter, captured, idleLimit=3, movementThresh=100, dp=1, param1=100,param2=100):
    trigger = False
    rad =0
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    frameDelta = cv2.absdiff(prevGray, gray)
    tresh = cv2.threshold(frameDelta, movementThresh, 255, cv2.THRESH_BINARY)[1]
    
    #%% Add erosion

    if not (tresh>0).any(): #No movement is detected
        idleCounter += 1 #Keep track of consecutive frames with no movement
    else:
        idleCounter = 0 #Reset when movement is detected
        captured = False
    if idleCounter == idleLimit and captured == False:
        #The frame is now idle for the specified time. Detect circle
        captured = True
        #Check if circle is visible
        minDistCircle = 0.1 * gray.shape[1]#Minimum distance between two image centers
        minR = int(0.3*gray.shape[1])
        maxR = int(0.47*gray.shape[1])
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,dp,minDistCircle,
                                param1=param1,param2=param2,minRadius=minR,maxRadius=maxR)
            
        if not type(circles) == type(None):
            #Analyze the image further
            trigger = True
            rad = circles[0][0][2]/gray.shape[1]
    return trigger, gray, idleCounter, captured, rad


def polarTransform(I, center, r, theta= (0,360), rstep = 1.0, thetastep = 360.0/(180*8)):
    #Get the range of minimum and maximum distance
    minr, maxr = r
    cx, cy = center
    mintheta, maxtheta = theta
    H = int((maxr - minr) / rstep) +1
    W = int((maxtheta - mintheta) / thetastep) +1
    O = 125 * np.ones((H, W), I.dtype)
    
    w,h = I.shape
    r = np.linspace(minr,maxr,H)            
    r = np.tile(r,(W,1))
    r = np.transpose(r)
    theta = np.linspace(mintheta,maxtheta,W)
    theta = np.tile(theta,(H,1))
    x,y=cv2.polarToCart(r,theta,angleInDegrees=True)


    #Nearest neighbor interpolation
    for i in range(H):
        for j in range(W):
            px = int(round(x[i][j])+cx)
            py = int(round(y[i][j])+cy)
            if((px >= 0 and px <= w-1) and (py >= 0 and py <= h-1)):
                O[i][j] = I[py][px]
            else:
                O[i][j] = 255
    return O 




def doPolarTransform(img, backgroundImg,minAreaSize, debug=False):
    if debug:
        plt.figure()
        plt.imshow(img)
    
    #%% Parameters
    param1 = 100
    param2 = 100
    dp = 1
    fovY = 540 #Size of the FOV in y direction, given in cm
    roiW = 30 #Desired ROI around the calculated radius, given in cm
    marginRadius = roiW/fovY * img.shape[1]
    
    
    #%% Get  middlePoint and radius
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9,9), 2)
    
    minDistCircle = 0.1 * gray.shape[1]#Minimum distance between two image centers
    minR = int(0.3*gray.shape[1])
    maxR = int(0.47*gray.shape[1])
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,dp,minDistCircle,
                            param1=param1,param2=param2,minRadius=minR,maxRadius=maxR)
    circles = circles[0]
    
    centerCoord = circles[0][0:2]
    radius = circles[0][2]
    radiusRange = (int(radius-marginRadius/2), int(radius + marginRadius/2))
    
    
    
    #%% Transform images to Polar, and calculate distance between them
    
    
    blurMag = (31,31)
    imgForPolar = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    bgForPolar = cv2.cvtColor(backgroundImg, cv2.COLOR_RGB2GRAY)
    
    
    frameDelta = cv2.absdiff(imgForPolar, bgForPolar)
    thresh = cv2.threshold(frameDelta, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

    
    polarThresh = polarTransform(thresh, tuple(centerCoord.astype(int)), radiusRange)
    
    
    

    if debug:
        plt.figure()
        plt.imshow(frameDelta, cmap='gray')
        plt.title("Difference image before polar")
        
        plt.figure()
        plt.imshow(thresh, cmap='gray') 
        plt.title("Otsu thresholded image")
        
        plt.figure()
        plt.imshow(polarThresh, cmap='gray')
        plt.title("Polar thresholded image")
        
            
    #Erode
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    polarThresh = cv2.erode(polarThresh,kernel,iterations = 1)
    

    
    
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(polarThresh, connectivity=4)
    sizes = stats[:, -1]
    
    
    validLabel = np.array(np.where(sizes>minAreaSize)) + 1
    validLabels = validLabel[0]
    
    img2 = np.zeros(output.shape)
    for validLabel in validLabels:
        img2[output == validLabel] = 255
    polarThresh = img2
    
    if debug:
        plt.figure()
        plt.imshow(polarThresh,cmap='gray')
        plt.title("Thresholded image after morphology")
    
    

    
    
    #%% Extract height for every x
    
    widthProfile = []
    for i in range(0,polarThresh.shape[1]):
    
        minW = np.argmax(polarThresh[:,i]>0)
        maxW = polarThresh[:,i].shape[0] - np.argmax(np.flip(polarThresh[:,i])>0)
        if minW == 0 or maxW == thresh[:,i].shape[0] or (maxW - minW ==1):
            widthProfile.append(0)
        else:
            widthProfile.append(maxW - minW)
    
            
        
    
    
    #%%
    #Put average where widthProfile is zero
    widthProfile = np.array(widthProfile)
    zeroCount = np.sum(widthProfile == 0)
    meanValue = np.sum(widthProfile) / (len(widthProfile) - zeroCount)
    widthProfile[np.where(widthProfile==0)] = meanValue
    
    
    #%% Filter
    
    winSize = 50
    
    filteredWidth = np.concatenate((widthProfile[-winSize:], widthProfile, widthProfile[0:winSize]))
    
    filteredWidth = np.convolve(filteredWidth, np.ones((winSize,))/winSize, mode='same')
    filteredWidth = filteredWidth[winSize:-winSize]    
    
    maxWidth = np.max(filteredWidth)
    meanWidth = np.mean(filteredWidth)
    diffWidth = maxWidth - meanWidth
    


        
        

    
    if debug:
        plt.figure()
        plt.plot(widthProfile)
        plt.xlim(0,len(widthProfile))
        plt.ylim(0,200)
        plt.title("Unfiltered widthProfile")
        
    
    
    if debug:
        plt.figure()
        plt.subplot(2,1,1)
        plt.imshow(polarThresh,cmap='gray')
        plt.title("Thresholded image after morphology")
        
        
        
        plt.subplot(2,1,2)
        plt.plot(filteredWidth)
        plt.xlim(0,len(filteredWidth))
        plt.ylim(0,200)
        plt.title("Filtered widthProfile")

    #Try to subtract the polar transform of the background image as well.
    #Use histogram equalization in areas of the rail
    return filteredWidth, maxWidth, meanWidth, diffWidth

    