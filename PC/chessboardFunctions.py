import matplotlib.pyplot as plt
import os
import cv2
import pandas as pd
import numpy as np

# %% Visualization function
    
def plotResult(calibrationImage, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts, debug=True):
    FOV = np.max(distance[distIdx[1:numPts+1]]) + 0.2*np.max(distance[distIdx[1:numPts+1]])
    #Area of the image to show, around the queryPt
    fovYStart = max(queryPt[1]-FOV, 0)
    fovYEnd = min(calibrationImage.shape[0], queryPt[1]+FOV)
    fovXStart = max(0, queryPt[0]-FOV)
    fovXEnd = min(calibrationImage.shape[1], queryPt[0]+FOV)
    
    #%% Show candidaites
    if debug:
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.axis('off')
        
        for i in range(len(selectedPoints)):
            plt.scatter(selectedPoints[i,0], selectedPoints[i,1])
            plt.annotate(i, (selectedPoints[i,0], selectedPoints[i,1]), color='r')
            
        plt.scatter(queryPt[0], queryPt[1], c='r', s=100)
        plt.ylim(fovYEnd, fovYStart)
        plt.xlim(fovXStart, fovXEnd)
    
    
    #%% Show classified points
    plt.figure()
    plt.imshow(calibrationImage, cmap='gray')
    plt.axis('off')
    for i, point in enumerate(classifiedPoints):
        plt.scatter(point[0], point[1])
        plt.annotate(pointNames[i], (point[0], point[1]), color='r')
    
    plt.scatter(queryPt[0], queryPt[1], c='r', s=100)
    plt.ylim(fovYEnd, fovYStart)
    plt.xlim(fovXStart, fovXEnd)


def findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDist, angleThresh, debug=False):
    """
    Selects a points from a list of candidate points which lies within a 
    specified angle range. The closest point satisfying this relation is returned
    """
    complete = True
    minDistStart = minDist
    mindiff = minDist
    p = np.zeros((1,2))
    iRemove = []
    ap = 720
    imin = 0
    for i, angle in enumerate(angles):
        if pointNum == 4: #Account for 360 degree flip point
            diff = min(baseAngle-angle, angle)
        else:
            diff = baseAngle-angle
        if np.abs(diff) <= angleThresh:
            if distanceCorners[i] < minDist:
                if minDist != minDistStart:
                    iRemove.append(imin)
                imin = i
                minDist = distanceCorners[i]
                p = keepCorners[i,:]
                mindiff = diff
                ap = angle
    if np.sum(p) == 0:
        if debug:
            print("P{} is not found".format(pointNum))
        complete= False
    
    return complete, p, iRemove, mindiff, ap, imin
        
def getAngle(queryPt, otherCorner, debug=False):
    """
    Calculates the angles between two points
    
    """
    diff = queryPt- otherCorner
    if debug:
        print("Query point (x,y)")
        print(queryPt)
        print("Corner (x,y)")
        print(otherCorner)
    angle = np.arctan2(-diff[1], diff[0])
    angle = angle*180/np.pi + 180
    return angle


def getsquareNums(shapeGrid):
    """
    Generates a list of coordinates for the left top coordinate of the squares
    on the rectified grid
    """
    squareNums = np.zeros((len(shapeGrid)*np.max(shapeGrid),2),dtype='int')
    counter = 0
    for i in range(len(shapeGrid)):
        for j in range(shapeGrid[i]):
            squareNums[counter,:] = [j,i]
            counter +=1
    squareNums = squareNums[0:counter,:]
    
    return squareNums

def getGridCorners(squareNum, squareNums, pps):
    """
    Find four corners of the square on the rectified grid

    """
    square = squareNums[squareNum,:]
    p1 = np.array([square[1]*pps, square[0]*pps], dtype='int')
    p2 = np.array([p1[0]+ pps, p1[1]], dtype='int')
    p3 = np.array([p2[0], p1[1] + pps], dtype='int')
    p4 = np.array([p1[0], p3[1]], dtype='int')
    gridPoints = np.vstack((p1,p2,p3,p4))
    return gridPoints


def detectChessBoardCorners(img, qualityLevel, minDistance, winSize, criteria):
    """
    Detect raw corners in the checkerboard

    """
    corners = cv2.goodFeaturesToTrack(img, 0, qualityLevel, minDistance)
    #Refine to subpixel
    corners = cv2.cornerSubPix(img, corners, winSize, (-1,-1), criteria)
    #Store in simple array
    corners = corners[:,0,:]
    return corners




def getMargins(gridMiddle, pps, outputGrid):
    """
    Calculate the number of squares from the detected middlePoint
    """
    margin_top = int(gridMiddle[0,1]/pps)
    margin_bottom = int((outputGrid.shape[0]- gridMiddle[3,1])/pps)
    margin_left = int(gridMiddle[0,0]/pps)
    margin_right = int((outputGrid.shape[1] - gridMiddle[1,0]) /pps)
    margins =  [margin_top, margin_bottom, margin_left, margin_right]
    return margins



def getTransforms(fixedPoints, squareList):
    """
    Estimate transformations using the matching point pair
    """
    
    tforms = [] 
    for i in range(len(squareList)):
        imagePoints = squareList[i]
        tform, _ = cv2.estimateAffine2D(imagePoints, fixedPoints)
        tforms.append(tform)
    return tforms



def transformImage(tforms,img, outputImg, pps, validSquareNum, squareNums):
    """
    Transform squares and place them on the outputGrid
    """
    for i in range(len(tforms)):
        dst = cv2.warpAffine(img, tforms[i], (pps, pps))
        gridPoints = getGridCorners(validSquareNum[i], squareNums, pps)
        outputImg[gridPoints[0,1]:gridPoints[3,1], gridPoints[0,0]:gridPoints[1,0],:] = dst
    return outputImg




def findRedPoint(img, debug=False):
    """
    Detect the middlePoint in the images
    """
    #Red is between H: 0-10, and 170-180
    
    wid, hei, _ = img.shape
    img = img[0:int(hei/2),:,:]
    hsvImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,100,80])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsvImage, lower_red, upper_red)
    #Range for upper range
    lower_red = np.array([170,100,80])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsvImage,lower_red,upper_red)

    
    mask = mask1+mask2
    
    

        
    if debug:
        plt.figure()
        plt.imshow(img)
        plt.figure()
        plt.imshow(hsvImage)
        plt.figure()
        plt.imshow(mask)
        
    #Morphology operations to refine mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    mask = cv2.erode(mask,kernel,iterations = 2)
    
    if debug:
        plt.figure()
        plt.imshow(mask, cmap='gray')
        
        
    red = np.fliplr(np.array(np.where(mask==255)).T)
    
    #%% Select max Y 
    maxY = red[np.where(red[:,1] == max(red[:,1]))]
    
    point = maxY[np.argmin(maxY[:,0])]
    
    
    
    return point


def fuseImages(rectifiedImages, middleSquares, outputGrids, pps, outMargins, fuseRatio, debug=True):
    """
    Fuse two images into one

    """
    rectifiedImages[0] = np.fliplr(np.flipud(rectifiedImages[0]))
    
    middleSquareC1 = middleSquares[0].copy()
    middleSquareC2 = middleSquares[1].copy()
    
    middleSquareC1[:,1] = (rectifiedImages[0].shape[0]) - middleSquareC1[:,1] #FlipUD
    middleSquareC1[:,0] = (rectifiedImages[0].shape[1]) - middleSquareC1[:,0] #FlipLR
    middleSquareC1 = np.vstack((middleSquareC1[2,:], middleSquareC1[3,:], middleSquareC1[0,:], middleSquareC1[1,:])) #Reorder 
    
    #After preprocessing
    c1margins = getMargins(middleSquareC1, pps, outputGrids[0])
    c2margins = getMargins(middleSquareC2, pps, outputGrids[1])
    

    c1_rectified = rectifiedImages[0]
    c2_rectified = rectifiedImages[1]

    c1_rectified = c1_rectified[:-1,:-1,:] #Shift images by one to allign them better.
    c2_rectified = c2_rectified[1:,1:,:] #This follows from the fact that the boundary is missing
    
    
    #%% Calculation of output size of the full fuse frame
    
    maxMargins = np.max(np.vstack((c1margins,c2margins)), axis=0)
    
    diff = (maxMargins[1] - maxMargins[0])
    if diff >=0:
        compensateMargin = [diff,0,0,0]
    else:
        compensateMargin = [0,-diff,0,0]
        
    maxMargins = maxMargins + compensateMargin
    
    verticalPixs = (maxMargins[0] + maxMargins[1]+ 1)*pps
    horizontalPixs = (maxMargins[2] + maxMargins[3]+ 1)*pps
    fuseFrame = np.zeros((verticalPixs, horizontalPixs,3), dtype='uint8')
    
    
    #%% Paste c1 and c2 into seperate fuseFrames
    
    c1_offset = (maxMargins - c1margins)*pps
    c2_offset = (maxMargins - c2margins)*pps
    c2_offset[0] = c2_offset[0]

    #Place images in the fuseFrame
    c1FuseFrame = fuseFrame.copy()
    
    c1FuseFrame[c1_offset[0]:c1_offset[0] + c1_rectified.shape[0],
                c1_offset[2]:c1_offset[2] + c1_rectified.shape[1],:] = c1_rectified
    

    c2FuseFrame = fuseFrame.copy()
    c2FuseFrame[c2_offset[0]:c2_offset[0] + c2_rectified.shape[0],
                c2_offset[2]:c2_offset[2] + c2_rectified.shape[1],:] = c2_rectified
    

    
    if debug:
        plt.figure()
        plt.imshow(c1FuseFrame)
        plt.figure()
        plt.imshow(c2FuseFrame)
        
    
    
    #%% Paste C1 into C2 to get the fused image
    c2FuseFrame[0:int(c1FuseFrame.shape[0]/fuseRatio),:,:] = c1FuseFrame[0:int(c1FuseFrame.shape[0]/fuseRatio),:,:]
    
    
    
    #%% Crop the fused image to fit the desired output size
    startY = (maxMargins[0] - outMargins[0])*pps
    endY = (maxMargins[0] + 1 + outMargins[1])*pps - 1
    startX  = (maxMargins[2] - outMargins[2])*pps
    endX    = (maxMargins[2] + 1 + outMargins[3])*pps - 1
    c2FuseFrame = c2FuseFrame[startY:endY,startX:endX,:]
    
    return c2FuseFrame

def getMiddleSquareIndex(middlePoint, squareList):
    """
    Finds the index of the square which is closest to the middlepoint

    """
    minDist = 1000
    for i, square in enumerate(squareList):
        dist = np.mean(np.linalg.norm(square - middlePoint,axis=1))
        if dist<minDist:
            imin = i
            minDist = dist
    return imin


def findSquares(image, queryPt, corners, numPts, angleThreshHorizontal, angleThreshVertical, debug=False):
    """
    Checks if a query point is a valid one, by checking if its the shared corner
    of four squares

    """
    minDistStart = 1000
    #Calculate distance between query point and other points
    distance = np.linalg.norm(corners-queryPt, axis=1)
    distIdx = np.argsort(distance)
    #Keep X closest points
    keepCorners = corners[distIdx[1:numPts+1],:]
    selectedPoints = keepCorners.copy()
    distanceCorners = distance[distIdx[1:numPts+1]]
    
    #%% Start of classification
    #Find angles with respect to query point
    iRemove = []
    pointNames = []
    complete = True
    classifiedPoints = []
    #Calculate the angles between the query point and the other points
    angles = []
    for i in range(len(keepCorners)):
        angle = getAngle(queryPt, keepCorners[i,:])
        angles.append(angle)
    
    
    #%% Classification of P4
    #Select point which are close to 360 or 0 degrees
    baseAngle = 360
    angleThresh = angleThreshHorizontal
    pointNum = 4
    
    complete, p, idel, mindiff, ap04, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    
    ap04 = min(ap04, 360-ap04)
    iRemove = iRemove + idel
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    #%% P8
    baseAngle = 180 - ap04
    angleThresh = angleThreshHorizontal
    pointNum = 8
    
    complete, p, idel, mindiff, ap08, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    iRemove = iRemove + idel
    if complete:    
        classifiedPoints.append(p)
        pointNames.append(pointNum)
        iRemove.append(imin)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    #%% P2
    baseAngle = 90
    angleThresh = 45
    pointNum = 2
    minDist = minDistStart
    
    angleRangeMin = baseAngle - angleThresh
    angleRangeMax = baseAngle + angleThresh
    
    # Start of "Function" for p2
    # Find angles between angleRangeMin
    idel = []
    p = np.zeros((1,2))
    for i, angle in enumerate(angles):
        if angle >= angleRangeMin and angle <= angleRangeMax:
            #Angle is within the specified range, keep the closest
             if distanceCorners[i] < minDist:
                if minDist != minDistStart:
                    idel.append(imin)
                imin = i
                minDist = distanceCorners[i]
                p = keepCorners[i,:]
                ap02 = angle #Angle between P0 and P2.
           
    iRemove = iRemove + idel 
    if np.sum(p) == 0:
        complete= False
    
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    
    
    #%% P6
    baseAngle = 270 - (90 - ap02)
    angleThresh = angleThreshHorizontal
    pointNum = 6
    
    complete, p, idel, mindiff, ap06, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    iRemove = iRemove + idel
    if complete:    
        classifiedPoints.append(p)
        pointNames.append(pointNum)
        iRemove.append(imin)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    

    
    #%% Clean points which are used now
    #Clean from: distanceCorners,  angles and keepCorners
    distanceCorners = np.delete(distanceCorners, iRemove)
    angles = [j for i, j in enumerate(angles) if i not in iRemove]
    keepCorners = np.delete(keepCorners, iRemove, axis=0)
    
    #%% P3 and P5
     
    angles4 = []
    queryPt4 = classifiedPoints[0]
    for i in range(len(keepCorners)):
        angle = getAngle(queryPt4, keepCorners[i,:])
        #print("Angle(Query, P({}) = {:.2f}".format(i,angle))
        angles4.append(angle)
        
    # P3
    baseAngle = ap02
    angleThresh = angleThreshVertical
    pointNum = 3
    
    complete, p, idel, mindiff, ap43, imin = findPoint(pointNum, baseAngle, angles4, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    

    # P5
    baseAngle = 270 - (90 - ap43)
    angleThresh = angleThreshVertical
    pointNum = 5
    
    complete, p, idel, mindiff, ap45, imin = findPoint(pointNum, baseAngle, angles4, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    #%% P1 and P7
    
    angles8 = []
    queryPt8 = classifiedPoints[1]
    for i in range(len(keepCorners)):
        angle = getAngle(queryPt8, keepCorners[i,:])
        #print("Angle(Query, P({}) = {:.2f}".format(i,angle))
        angles8.append(angle)
        
    # P1
    baseAngle = ap02
    angleThresh = angleThreshVertical
    pointNum = 1
    
    complete, p, idel, mindiff, ap81, imin = findPoint(pointNum, baseAngle, angles8, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    # P7
    baseAngle = 270 - (90 - ap81)
    angleThresh = angleThreshVertical
    pointNum = 7
    
    complete, p, idel, mindiff, ap87, imin = findPoint(pointNum, baseAngle, angles8, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
    if complete:
        iRemove.append(imin)
        classifiedPoints.append(p)
        pointNames.append(pointNum)
    else:
                
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        return complete, np.array(classifiedPoints), pointNames
    
    #%% END OF Point classification. Now check the square for its validity
    
    a71 = getAngle(classifiedPoints[7], classifiedPoints[6])
    a75 = getAngle(classifiedPoints[7], classifiedPoints[5])
    
    a15 = a71+a75
    
    #alculate angle between 7 and 3
    a17 = getAngle(classifiedPoints[6], classifiedPoints[7])
    a13 = getAngle(classifiedPoints[6], classifiedPoints[4])
    
    a37 = 180 - (360 - a17) + a13
    
    diff = np.abs(a15 - a37)
    
    
    diff = min(diff, 360-diff)
    if np.abs(diff) > 4:
        
        print("Square check failed")
        complete = False
        #plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts, debug=True)
        


    if debug:
        plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
        
        
    return complete, np.array(classifiedPoints), pointNames


def findValidPoints(calibrationImage, corners, numPts, angleThreshHorizontal, angleThreshVertical, checkQuery=True):
    """
    Loops through all corner candidates. Returns only point which are part of
    a valid square.
    """
    validPoints = np.zeros((2,1))
    validQuery = np.zeros((2,1))
    for i in range(len(corners)):
        queryPt = corners[i,:]
        complete, classifiedPoints, pointNames  = findSquares(calibrationImage, queryPt, corners, numPts, angleThreshHorizontal, angleThreshVertical)
        if complete:
            if (validPoints == 0).all():
                validPoints = classifiedPoints
                validQuery = queryPt
            else:
                validPoints = np.vstack((validPoints, classifiedPoints))
                validQuery = np.vstack((validQuery, queryPt))
    return np.unique(validPoints,axis=0) 


def getLTQuery(queryPoints, middlePoint, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical):
    """ Finds left top Query Point by 'walking' around the grid using the valid squares
        starting from the middle of the image"""
    distance = np.linalg.norm(queryPoints-middlePoint, axis=1)
    middleQuery = np.argsort(distance)[0]
    queryPt = queryPoints[middleQuery,:]
    middleQuery = queryPt.copy()
    while True:
        complete, classifiedPoints, pointNames  = findSquares(calibrationImage, queryPt, validPoints, numPts, angleThreshHorizontal, angleThreshVertical)
        #Check if go left
        if classifiedPoints[1,:] in queryPoints: #Check if left is available
            queryPt = classifiedPoints[1,:]
        elif classifiedPoints[2,:] in queryPoints: #Else go up
            queryPt = classifiedPoints[2,:]
        elif classifiedPoints[4,:] in queryPoints: #Else go up
            queryPt = classifiedPoints[4,:]
        else:
            break
        
    return queryPt, middleQuery


def getSquareList(queryPt, queryPoints, middleQuery, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=False):
    """
    Maps corners to a grid, by using topological relation between the squares.
    
    """
    try:
        rowCounts = []
        squareList = []
        rowCount = 0
        left = True
        switch = False
        lastRow = False
        startNext = 0
        count = 0
        
        startQueryPt = queryPt.copy()
        while True:
            complete, classifiedPoints, pointNames  = findSquares(calibrationImage, queryPt, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=debug)
            if rowCount ==0:
                if (classifiedPoints[0,:] == queryPoints).all(axis=1).any(): #If right is available
                        startPrev = startNext
                        startNext = classifiedPoints[0,:]
                else:
                    lastRow = True
                if debug:
                    plt.figure()
                    plt.imshow(calibrationImage)
                    for square in squareList:
                        plt.scatter(square[:,0], square[:,1])
    
            if left: #On the left side of the image, the LT and LB corners should be stored
                 #Store LT
                squareList.append(np.vstack((classifiedPoints[6,:], classifiedPoints[2,:], queryPt, classifiedPoints[1,:])))
                rowCount +=1
                
                #Go down
                if (classifiedPoints[3,:] == queryPoints).all(axis=1).any(): #Check if down is available
                    queryPt = classifiedPoints[3,:]
                else:
                    #Store LB
                    squareList.append(np.vstack((classifiedPoints[1,:], queryPt, classifiedPoints[3,:], classifiedPoints[7,:]))) #Store LT
                    rowCount +=1
                    rowCounts.append(rowCount)
                    rowCount = 0
                    
                    if not switch:
                        queryPt = startNext
                    else:
                        queryPt = startPrev
                        left = False
                        switch = False
                        bigEnough = int(1.75*len(rowCounts))
                        
                    if lastRow: #Last row is triggered too early, lower start point
                    
                        complete, classifiedPoints, pointNames  = findSquares(calibrationImage, startQueryPt, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=debug)
                        if (classifiedPoints[3,:] == queryPoints).all(axis=1).any(): #Check if down is available
                          newStart = classifiedPoints[3,:]
                        elif (classifiedPoints[5,:] == queryPoints).all(axis=1).any(): #Check if down is available
                          newStart = classifiedPoints[5,:]
                        return getSquareList(newStart, queryPoints, middleQuery, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=debug)
                        
                        
                    
                        
                    
            else: #On the right side of the image, the RT and RB corners should be stored
                squareList.append(np.vstack((classifiedPoints[2,:], classifiedPoints[4,:], classifiedPoints[0,:], queryPt,))) # Store RT
                rowCount +=1
                if (classifiedPoints[3,:] == queryPoints).all(axis=1).any(): #Check if down is available
                    queryPt = classifiedPoints[3,:]
                else:
                    squareList.append(np.vstack((queryPt, classifiedPoints[0,:], classifiedPoints[5,:], classifiedPoints[3,:]))) #Store RB
                    rowCount +=1
                    queryPt = startNext
                    rowCounts.append(rowCount)
                    rowCount = 0
                    if lastRow:
                        if len(rowCounts)> bigEnough:
                            break
                        else:
                            complete, classifiedPoints, pointNames  = findSquares(calibrationImage, startQueryPt, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=debug)
                            newStart = classifiedPoints[3,:]
                            return getSquareList(newStart, queryPoints, middleQuery, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical, debug=debug)
                        
                        
                
            if (queryPt==middleQuery).all(): #Switch from left mode to right mode
                #Startpoint should not change this time
                switch = True
            
        return squareList, rowCounts
    
    except Exception as e: 
        print(e)
        print(queryPt)
        
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.scatter(validPoints[:,0], validPoints[:,1])
        plt.scatter(queryPoints[:,0], queryPoints[:, 1], c='r')
        plt.scatter(queryPt[0], queryPt[1], c='g')
        
        
    


def calibrateCamera(calibrationImage, middlePoint, minDistance, qualityLevel, criteria, winSize, pps, numPts, angleThreshHorizontal, angleThreshVertical, debug=True):
    """
    Finds the transformations from the squares to a rectified grid

    """
    
    calibrationImage = cv2.cvtColor(calibrationImage, cv2.COLOR_BGR2GRAY)
    corners = detectChessBoardCorners(calibrationImage, qualityLevel, minDistance, winSize, criteria)
    
    if debug:
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.scatter(corners[:,0], corners[:,1])
        plt.title("Raw corners")
        
    #Find valid points and valid query points
    validPoints = findValidPoints(calibrationImage, corners, numPts, angleThreshHorizontal, angleThreshVertical)
    validQuery = []
    for i in range(len(validPoints)):
        queryPt = validPoints[i,:]
        complete, classifiedPoints, pointNames  = findSquares(calibrationImage, queryPt, validPoints, numPts, angleThreshHorizontal, angleThreshVertical)
        if complete:
            validQuery.append(i)
    queryPoints = validPoints[validQuery,:]
        
        
    if debug:
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.scatter(validPoints[:,0], validPoints[:,1])
        plt.title("Valid points")
            
        
        
    if debug:
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.scatter(validPoints[:,0], validPoints[:,1])
        plt.scatter(queryPoints[:,0], queryPoints[:, 1], c='r')
        plt.title("Valid query points")
    
    
    
    
    
    #Find left top query point
    LTQuery, middleQuery = getLTQuery(queryPoints, middlePoint, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical)
    
    if debug:
        plt.figure()
        plt.imshow(calibrationImage, cmap='gray')
        plt.scatter(validPoints[:,0], validPoints[:,1])
        plt.scatter(queryPoints[:,0], queryPoints[:, 1], c='r')
        plt.scatter(LTQuery[0], LTQuery[1], c='g')
        plt.scatter(middleQuery[0], middleQuery[1], c='y')
        plt.title("Left top query point and middleQuery")
        
    squareList, rowCounts = getSquareList(LTQuery, queryPoints, middleQuery, calibrationImage, validPoints, numPts, angleThreshHorizontal, angleThreshVertical,debug=False)    
    
    #%% Generate output grid
    
    
    squareRows = np.max(rowCounts)
    squareCols = len(rowCounts)
    shapeGrid = [squareRows]*squareCols
    squareNums = getsquareNums(shapeGrid)
    
    outputGrid = np.zeros((squareRows*pps,  squareCols*pps, 3),dtype='uint8')



        
    #%%Generate valid Square num
    #validSquareNum contains the indexes of the squares for which a transformation
    #should be estimated
    start = 0
    for i, row in enumerate(rowCounts):
        start = i*squareRows
        if i==0:
            validSquareNum = list(range(start,start+row))
        else:
            validSquareNum = validSquareNum + list(range(start,start+row))
        
        
        
    fixedPoints = getGridCorners(0, squareNums, pps)
    tforms = getTransforms(fixedPoints, squareList)
    return tforms, outputGrid, validSquareNum, squareNums, corners, squareList


    
    
    
def findSquaresTmp(image, analysisPt, queryPt, corners, numPts, angleThreshHorizontal, angleThreshVertical, debug=False):
    """
    Checks if a query point is a valid one, by checking if its the shared corner
    of four squares

    """
    minDistStart = 1000
    #Calculate distance between query point and other points
    distance = np.linalg.norm(corners-queryPt, axis=1)
    distIdx = np.argsort(distance)
    #Keep X closest points
    keepCorners = corners[distIdx[1:numPts+1],:]
    selectedPoints = keepCorners.copy()
    distanceCorners = distance[distIdx[1:numPts+1]]
    
    
    if (keepCorners == analysisPt).all(axis=1).any():
        #%% Start of classification
        #Find angles with respect to query point
        iRemove = []
        pointNames = []
        complete = True
        classifiedPoints = []
        #Calculate the angles between the query point and the other points
        angles = []
        for i in range(len(keepCorners)):
            angle = getAngle(queryPt, keepCorners[i,:])
            angles.append(angle)
        
        
        #%% Classification of P4
        #Select point which are close to 360 or 0 degrees
        baseAngle = 360
        angleThresh = angleThreshHorizontal
        pointNum = 4
        
        complete, p, idel, mindiff, ap04, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        
        ap04 = min(ap04, 360-ap04)
        iRemove = iRemove + idel
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        #%% P8
        baseAngle = 180 - ap04
        angleThresh = angleThreshHorizontal
        pointNum = 8
        
        complete, p, idel, mindiff, ap08, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        iRemove = iRemove + idel
        if complete:    
            classifiedPoints.append(p)
            pointNames.append(pointNum)
            iRemove.append(imin)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        #%% P2
        baseAngle = 90
        angleThresh = 45
        pointNum = 2
        minDist = minDistStart
        
        angleRangeMin = baseAngle - angleThresh
        angleRangeMax = baseAngle + angleThresh
        
        # Start of "Function" for p2
        # Find angles between angleRangeMin
        idel = []
        p = np.zeros((1,2))
        for i, angle in enumerate(angles):
            if angle >= angleRangeMin and angle <= angleRangeMax:
                #Angle is within the specified range, keep the closest
                 if distanceCorners[i] < minDist:
                    if minDist != minDistStart:
                        idel.append(imin)
                    imin = i
                    minDist = distanceCorners[i]
                    p = keepCorners[i,:]
                    ap02 = angle #Angle between P0 and P2.
               
        iRemove = iRemove + idel 
        if np.sum(p) == 0:
            complete= False
        
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        
        
        #%% P6
        baseAngle = 270 - (90 - ap02)
        angleThresh = angleThreshHorizontal
        pointNum = 6
        
        complete, p, idel, mindiff, ap06, imin = findPoint(pointNum, baseAngle, angles, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        iRemove = iRemove + idel
        if complete:    
            classifiedPoints.append(p)
            pointNames.append(pointNum)
            iRemove.append(imin)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
    
        
        #%% Clean points which are used now
        #Clean from: distanceCorners,  angles and keepCorners
        distanceCorners = np.delete(distanceCorners, iRemove)
        angles = [j for i, j in enumerate(angles) if i not in iRemove]
        keepCorners = np.delete(keepCorners, iRemove, axis=0)
        
        #%% P3 and P5
         
        angles4 = []
        queryPt4 = classifiedPoints[0]
        for i in range(len(keepCorners)):
            angle = getAngle(queryPt4, keepCorners[i,:])
            #print("Angle(Query, P({}) = {:.2f}".format(i,angle))
            angles4.append(angle)
            
        # P3
        baseAngle = ap02
        angleThresh = angleThreshVertical
        pointNum = 3
        
        complete, p, idel, mindiff, ap43, imin = findPoint(pointNum, baseAngle, angles4, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
    
        # P5
        baseAngle = 270 - (90 - ap43)
        angleThresh = angleThreshVertical
        pointNum = 5
        
        complete, p, idel, mindiff, ap45, imin = findPoint(pointNum, baseAngle, angles4, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        #%% P1 and P7
        
        angles8 = []
        queryPt8 = classifiedPoints[1]
        for i in range(len(keepCorners)):
            angle = getAngle(queryPt8, keepCorners[i,:])
            #print("Angle(Query, P({}) = {:.2f}".format(i,angle))
            angles8.append(angle)
            
        # P1
        baseAngle = ap02
        angleThresh = angleThreshVertical
        pointNum = 1
        
        complete, p, idel, mindiff, ap81, imin = findPoint(pointNum, baseAngle, angles8, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        # P7
        baseAngle = 270 - (90 - ap81)
        angleThresh = angleThreshVertical
        pointNum = 7
        
        complete, p, idel, mindiff, ap87, imin = findPoint(pointNum, baseAngle, angles8, distanceCorners, keepCorners, minDistStart, angleThreshHorizontal)
        if complete:
            iRemove.append(imin)
            classifiedPoints.append(p)
            pointNames.append(pointNum)
        else:
                    
            if debug:
                plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            return complete, np.array(classifiedPoints), pointNames
        
        #%% END OF Point classification. Now check the square for its validity
        
        a71 = getAngle(classifiedPoints[7], classifiedPoints[6])
        a75 = getAngle(classifiedPoints[7], classifiedPoints[5])
        
        a15 = a71+a75
        
        #alculate angle between 7 and 3
        a17 = getAngle(classifiedPoints[6], classifiedPoints[7])
        a13 = getAngle(classifiedPoints[6], classifiedPoints[4])
        
        a37 = 180 - (360 - a17) + a13
        
        diff = np.abs(a15 - a37)
        
        
        diff = min(diff, 360-diff)
        if np.abs(diff) > 2:
            print("Square check failed")
            complete = False
            
            
    
        if debug:
            plotResult(image, selectedPoints, classifiedPoints, pointNames, queryPt, distance, distIdx, numPts)
            
            
        return complete, np.array(classifiedPoints), pointNames
    else:
        return False, 0,0
    
    
    
def imageREfuse(rectifiedImages, middleSquares, outputGrids, pps, outMargins, fuseRatio,debug=True):
    """
    Fuse two images into one

    """
    
    middleSquareC1 = middleSquares[0].copy()
    middleSquareC2 = middleSquares[1].copy()
    
    middleSquareC1[:,1] = (rectifiedImages[0].shape[0]) - middleSquareC1[:,1] #FlipUD
    middleSquareC1[:,0] = (rectifiedImages[0].shape[1]) - middleSquareC1[:,0] #FlipLR
    middleSquareC1 = np.vstack((middleSquareC1[2,:], middleSquareC1[3,:], middleSquareC1[0,:], middleSquareC1[1,:])) #Reorder 
    
    #After preprocessing
    c1margins = getMargins(middleSquareC1, pps, outputGrids[0])
    c2margins = getMargins(middleSquareC2, pps, outputGrids[1])
    

    c1_rectified = rectifiedImages[0]
    c2_rectified = rectifiedImages[1]

    c1_rectified = c1_rectified[:-1,:-1,:] #Shift images by one to allign them better.
    c2_rectified = c2_rectified[1:,1:,:] #This follows from the fact that the boundary is missing
    
    
    #%% Calculation of output size of the full fuse frame
    
    maxMargins = np.max(np.vstack((c1margins,c2margins)), axis=0)
    
    diff = (maxMargins[1] - maxMargins[0])
    if diff >=0:
        compensateMargin = [diff,0,0,0]
    else:
        compensateMargin = [0,-diff,0,0]
        
    maxMargins = maxMargins + compensateMargin
    
    verticalPixs = (maxMargins[0] + maxMargins[1]+ 1)*pps
    horizontalPixs = (maxMargins[2] + maxMargins[3]+ 1)*pps
    fuseFrame = np.zeros((verticalPixs, horizontalPixs,3), dtype='uint8')
    
    
    #%% Paste c1 and c2 into seperate fuseFrames
    
    c1_offset = (maxMargins - c1margins)*pps
    c2_offset = (maxMargins - c2margins)*pps
    c2_offset[0] = c2_offset[0]

    #Place images in the fuseFrame
    c1FuseFrame = fuseFrame.copy()
    
    c1FuseFrame[c1_offset[0]:c1_offset[0] + c1_rectified.shape[0],
                c1_offset[2]:c1_offset[2] + c1_rectified.shape[1],:] = c1_rectified
    

    c2FuseFrame = fuseFrame.copy()
    c2FuseFrame[c2_offset[0]:c2_offset[0] + c2_rectified.shape[0],
                c2_offset[2]:c2_offset[2] + c2_rectified.shape[1],:] = c2_rectified
    

    
    if debug:
        plt.figure()
        plt.imshow(c1FuseFrame)
        plt.figure()
        plt.imshow(c2FuseFrame)
        
    
    
    #%% Paste C1 into C2 to get the fused image
    c2FuseFrame[0:int(c1FuseFrame.shape[0]/fuseRatio),:,:] = c1FuseFrame[0:int(c1FuseFrame.shape[0]/fuseRatio),:,:]
    
    
    
    #%% Crop the fused image to fit the desired output size
    startY = (maxMargins[0] - outMargins[0])*pps
    endY = (maxMargins[0] + 1 + outMargins[1])*pps - 1
    startX  = (maxMargins[2] - outMargins[2])*pps
    endX    = (maxMargins[2] + 1 + outMargins[3])*pps - 1
    c2FuseFrame = c2FuseFrame[startY:endY,startX:endX,:]
    
    return c2FuseFrame











               
