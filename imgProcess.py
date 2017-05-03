# -*- coding: utf-8 -*-
"""
Created on Wed May  3 15:41:49 2017

@author: Huw
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 09:18:53 2017

@author: Nessa
"""
#Real function of canny.py. implement this file

import cv2
import random
import numpy as np
from scipy.spatial import distance
from scipy import spatial
from random import randint

def imgProcess():

    img = cv2.imread('test.jpg',0)    #NEED TO SAVE AS SAME THING EACH TIME, 0 converts to grayscale
    
    r = 400.0 / img.shape[1]
    dim = (400, int(img.shape[0] * r)) # Resize image 300 x 225 pixels params
    
    # perform the actual resizing of the image and show it
    kSize = 10
    kernel = np.ones((kSize,kSize),np.uint8)
    #smoothed = cv2.filter2D(img,-1,kernel) # Smooth image with 15x15 array
    opening = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    
    resized = cv2.resize(opening, dim, interpolation = cv2.INTER_AREA) # Resize image
    
    #cv2.imwrite('Resized.jpg',resized)
    sigma = 0.333
    v = np.median(resized)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v)) # Define automatic upper and lower thresholds for Canny detection
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(resized, lower, upper) # Perform Canny Edge Detection
    #edge_save = cv2.imwrite('edged.jpg',edges)
    
    
    
    #colr = cv2.imread('test.jpg')
    #smallColr= cv2.resize(colr, dim, interpolation = cv2.INTER_AREA) # Resized colour version of original image
    #cv2.imshow('Edges',edges)
    image,contours,hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #for i in range(len(contours)):
     #   color = np.random.rand(3) * 255
      #  cnt_img = cv2.drawContours(cnt_img,contours,i,color,3)
    
    from perspCorr2 import perspMatrix
    h = perspMatrix
    
    #cnt_img = cv2.warpPerspective(smallColr,h,(4000,3000)) #cnt_img is transformed image (transform small colour image)
    #cv2.imshow('Warped',cnt_img)
    #cv2.imshow('Original', smallColr)
     
    #Added finding contour work:
    #This assumes that the robot is already on the line
    #create array containing closest contours to robot
    
    
    contours_shorten = list(contours) # duplicate contours list and remove any contours that are too short
    
    for cont in range(len(contours)):
        if len(contours[cont]) < 19:
            contours_shorten.remove(contours[cont])
    
    contour_pertr = [] # perspective correction contours
    contours_pertrInts = []
    
    for cont3 in range(len(contours_shorten)):
        contours_shorten[cont3] = contours_shorten[cont3].reshape(-1,2)
        a = contours_shorten[cont3]
        a = np.array([a])
        a = a.astype(float)
        contour_pertr.append(cv2.perspectiveTransform(a,h))
        contour_pertr[cont3] = np.reshape(np.ravel(contour_pertr[cont3]),(-1,1,2))
        contour_pertr[cont3] = np.reshape(np.ravel(contour_pertr[cont3]),(-1,2))
        contours_pertrInts.append(cv2.perspectiveTransform(a,h))
        contours_pertrInts[cont3] = np.reshape(np.ravel(contours_pertrInts[cont3]),(-1,1,2))
        contours_pertrInts[cont3] = np.reshape(np.ravel(contours_pertrInts[cont3]),(-1,2))
        #contours_pertrInts[cont3] = contours_pertrInts[cont3].astype(int)
            #for ind in range(2):
            #    contours_pertrInts[cont3][cont_pertr][ind] = int(contour_pertr[cont3][cont_pertr][ind])
        #contours_pertrInts[cont3] = np.reshape(np.ravel(contours_pertrInts[cont3]),(-1,1,2))
        #contours_pertrInts[cont3].astype(int)
        #contour_pertr[cont3] = np.reshape(np.ravel(contour_pertr[cont3]),(-1,1,2))
        #contours_shorten[cont3] = a.reshape(-1,1,2)
    #contours = contours3
    #contour_pertr.astype(int)
    
    
           
    #cnt_img2 = cv2.drawContours(cnt_img,contours_pertrInts,-1,(0,255,0),3) #cnt_img2 has contours on it
    #cv2.imshow('Contours',cnt_img2)
    
    closestContours = []
    for i in range(len(contours)): #iterate through each contour
        for point in range(len(contours[i])): #iterate through the points in each contour
            # find contours with points in middle 165-235 pixels x and bottom 280-300 y
            if 100 <= contours[i][point][0][0] <= 300 and 280<= contours[i][point][0][1] <= 299 and len(contours[i])>=19: #thresholds can be changed to ensure just 2 contours are found
                closestContours.append(i) 
                break
    
    
    
    midpointx = []
    midpointy = []
    mptheta = []
    clcont = []
    
    if len(closestContours)==0: #Not on line, cannot see any lines to follow
         if len(contours) >= 1:
             for k  in range(len(contours)):
                 clcont.append(abs(cv2.pointPolygonTest(contours[k],(200,299),True)))
             followContour = clcont.index(min(clcont))
             randpoint = [0,0]
             p = [200,299]
             while distance.euclidean(p,randpoint) >= 150 or distance.euclidean(p,randpoint) <= 30:
                 randindex = randint(0,len(contours[followContour])-1)
                 randpoint = contours[followContour][randindex][0]
                
           
             for k in range(-2,2):
                 refindex = randindex + k
                 if 0 <= refindex < len(contours[followContour]):
                     midpointx.append(contours[followContour][refindex][0][0])
                     midpointy.append(contours[followContour][refindex][0][1])
                
             
             #plot midpoints
             midptList = np.array(list(zip(midpointx,midpointy)))
             midptList = midptList.reshape((-1,1,2))
             midptList = midptList.reshape(-1,2)
             b = midptList
             b = np.array([b])
             b = b.astype(float)
             midptList_dist = cv2.perspectiveTransform(b,h)
             midptList_dist = midptList_dist.reshape((-1,1,2))
             #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0,255,255),1)
             #cv2.namedWindow('midpoints')
             #cv2.imshow('midpoints', cnt_img2)
             
             for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
                 mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]),(midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
             mptheta.append(mptheta[len(mptheta)-1])
             
    
    #Go to Nessa's code to move  robot to locate new contour
    
    if len(closestContours)==1: # Can only see one contour, choose points on this line to follow
        randpoint = [500,300]
        p = [0,0]
        while distance.euclidean(p,randpoint) >= 150 or distance.euclidean(p,randpoint) <= 30:
            randindex = randint(0,len(contours[closestContours[0]])-1)
            randpoint = contours[closestContours[0]][randindex][0]
            
        
       
        for k in range(-2,2):
            refindex = randindex + k
            if 0 <= refindex < len(contours[closestContours[0]]):
                midpointx.append(contours[closestContours[0]][refindex][0][0])
                midpointy.append(contours[closestContours[0]][refindex][0][1])
            
        
        #plot midpoints
        midptList = np.array(list(zip(midpointx,midpointy)))
        midptList = midptList.reshape((-1,1,2))
        
        midptList = midptList.reshape(-1,2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b,h)
        midptList_dist = midptList_dist.reshape((-1,1,2))
        #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0,255,255),1)
        #cv2.namedWindow('midpoints')
        #cv2.imshow('midpoints', cnt_img2)
        
        
        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]),(midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1])    
        
    if len(closestContours)==2:
        #Pick one of the two contours to follow at random
        followContour = random.choice(closestContours)
        print(followContour)
        dist2 = []
        #Find the other contour that is not being followed
        m = []
        contours2 = list(contours)
        contours2.remove(contours2[followContour])
        
        for contour in range(len(contours2)):
            if len(contours2[contour]) > 19:
                  contourpoints = np.reshape(np.ravel(contours2[contour]),(-1,2)).tolist()
                  m = m + contourpoints
        #followContourListPosition = closestContours.index(followContour)
        #otherContour = closestContours[abs(followContourListPosition-1)]
        #otherContourList = np.reshape(np.ravel(contours[otherContour]),(-1,2))
    
        
        #For each point in the followed contour, find the closest point in the other contour
        tree = spatial.KDTree(m)
        for pt in range(0,len(contours[followContour]),1):
            p = contours[followContour][pt][0]
            pointonLine = m[tree.query(p)[1]]
            lineDistance = distance.euclidean(p,pointonLine)
            if 5 <= lineDistance <= 130: #if distance between lines is around tape width - i.e. lines not diverging (may need to change)
                midpointx.append((p[0]+pointonLine[0])/2) # add midpoints
                midpointy.append((p[1]+pointonLine[1])/2)
    
    
     
        #plot midpoints
        midptList = np.array(list(zip(midpointx,midpointy)))
        midptList = midptList.reshape((-1,1,2))
        
        midptList = midptList.reshape(-1,2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b,h)
        midptList_dist = midptList_dist.reshape((-1,1,2))
        #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0,255,255),1)
        
        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]),(midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1])
        
        #cv2.namedWindow('midpoints')
        #cv2.imshow('midpoints', cnt_img2)
        #cv2.imwrite('midpts.jpg',cnt_img2)
    
    
    if len(closestContours)==3: #3 contours found in radius i.e. t junction
        dist3 = []
        randcont = []
    # ** Old Code **
    #    for cont3 in range(len(closestContours)):
    #        dist3.append(abs(cv2.pointPolygonTest(contours[closestContours[cont3]],(200,299),True)))
    #    max_dist_index = dist3.index(max(dist3))
    #    furthestcontour = closestContours[max_dist_index]
    #    randcont.append(furthestcontour) # always follow new contour (starts furthest away)
    #    closestContours.remove(furthestcontour)
    #    randcont.append(random.choice(closestContours))
    #    followContour = random.choice(randcont) # choose at random one of other two contours to follow
    #    followContourListPosition = randcont.index(followContour)
    #    otherContour = randcont[abs(followContourListPosition-1)] #choose at random which contour will be the main contour to follow
    #    otherContourList = np.reshape(np.ravel(contours[otherContour]),(-1,2))
    
        # **New Code **
        correctcontour = 0
        while correctcontour == 0:
            followContour = random.choice(closestContours)
            print(followContour)
           
            
            #Find the other contour that is not being followed
            for cnt3 in range(len(contours)):
                dist3 = []
                if cnt3 != followContour:
                    for c in range(0,len(contours[followContour]),round(len(contours[followContour])/10)):
                        fcPt = contours[followContour][c][0]
                        dist3.append(abs(cv2.pointPolygonTest(contours[cnt3],tuple(fcPt.tolist()),True)))
                    if all(10 <= dst <=130 for dst in dist3):
                        cnt3list = np.reshape(np.ravel(contours[cnt3]),(-1,2))
                        tree = spatial.KDTree(cnt3list)
                        pointonLine = cnt3list[tree.query(fcPt)[1]]
                        midx = int(round((fcPt[0]+pointonLine[0])/2))
                        midy = int(round((fcPt[1]+pointonLine[1])/2))
                        midpt = [midx,midy]
                        if resized[midy][midx] < 100:
                            otherContour = cnt3
                            correctcontour = 1
                            break
                      
                        
        
        otherContourList = np.reshape(np.ravel(contours[otherContour]),(-1,2))
                              
        #For each point in the followed contour, find the closest point in the other contour
    
        tree = spatial.KDTree(otherContourList)
        for pt in range(len(contours[followContour])):
            p = contours[followContour][pt][0]
            pointonLine = otherContourList[tree.query(p)[1]]
            lineDistance = distance.euclidean(p,pointonLine)
            if 5 <= lineDistance <= 150: #if distance between lines is around tape width - i.e. lines not diverging (may need to change)
                midpointx.append((p[0]+pointonLine[0])/2) # add midpoints
                midpointy.append((p[1]+pointonLine[1])/2)       
        
        
          
        #plot midpoints
        midptList = np.array(list(zip(midpointx,midpointy)))
        
        midptList = midptList.reshape(-1,2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b,h)
        midptList_dist = midptList_dist.reshape((-1,1,2))
        #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0,255,255),1)
        
        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]),(midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1])
        
        #cv2.namedWindow('midpoints')
        #cv2.imshow('midpoints', cnt_img2)
        
    
    
    if len(closestContours) >3:
        for k  in range(len(contours)):
            clcont.append(abs(cv2.pointPolygonTest(contours[k],(200,299),True)))
            followContour = clcont.index(min(clcont))
            randpoint = [0,0]
            p =[200,299]
            while distance.euclidean(p,randpoint) >= 150 or distance.euclidean(p,randpoint) <= 30:
                randindex = randint(0,len(contours[followContour])-1)
                randpoint = contours[followContour][randindex][0]
                
           
            for k in range(-2,2):
                refindex = randindex + k
                if 0 <= refindex < len(contours[followContour]):
                    midpointx.append(contours[followContour][refindex][0][0])
                    midpointy.append(contours[followContour][refindex][0][1])
               
            
             #plot midpoints
            midptList = np.array(list(zip(midpointx,midpointy)))
            midptList = midptList.reshape((-1,1,2))
            midptList = midptList.reshape(-1,2)
            b = midptList
            b = np.array([b])
            b = b.astype(float)
            midptList_dist = cv2.perspectiveTransform(b,h)
            midptList_dist = midptList_dist.reshape((-1,1,2))
            #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0,255,255),1)
            #cv2.namedWindow('midpoints')
            #cv2.imshow('midpoints', cnt_img2)
            
            for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
                mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]),(midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
            mptheta.append(mptheta[len(mptheta)-1])
            
    return [midptList_dist, mptheta]