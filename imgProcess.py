import cv2
import random
#import time
import numpy as np
from scipy.spatial import distance
from scipy import spatial
from random import randint

# Real function of canny.py. implement this file

#start_time = time.time()

def imgProcess(img):
    # NEED TO SAVE AS SAME THING EACH TIME, 0 converts to grayscale
    # img = cv2.imread('test.jpg', 0)
    # Resize image 300 x 225 pixels params
    r = 400.0 / img.shape[1]
    dim = (400, int(img.shape[0] * r))

    # perform the actual resizing of the image and show it
    kSize = 3
    kernel = np.ones((kSize, kSize), np.uint8)
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    # smoothed = cv2.filter2D(img, -1, kernel) # Smooth image with 15x15 array
    opening = cv2.morphologyEx(resized, cv2.MORPH_CLOSE, kernel)
    # Resize image
    

    # cv2.imwrite('Resized.jpg', resized)

    # apply automatic Canny edge detection using the computed median
    # Define automatic upper and lower thresholds for Canny detection
    v = np.median(opening)
    sigma = 0.333
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(opening, lower, upper)  # Perform Canny Edge Detection
    # edge_save = cv2.imwrite('edged.jpg', edges)

    # Resized colour version of original image
    smallColr= cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    # cv2.imshow('Edges', edges)

    # Find contours in canny image
    image, contours, hierarchy = cv2.findContours(edges,
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
    showcontours = cv2.drawContours(smallColr,contours,-1,(0, 255, 0),3)
    #cv2.imwrite('origcontours.jpg', showcontours)
    #cv2.imshow('original contours',showcontours)
    # for i in range(len(contours)):
    #     color = np.random.rand(3) * 255
    #     cnt_img = cv2.drawContours(cnt_img, contours, i, color, 3)

    from perspCorr2 import perspMatrix
    h = perspMatrix  # import perspective distortion matrix from other file

    # cnt_img is transformed image (transform small colour image)
    # cnt_img = cv2.warpPerspective(smallColr, h, (4000, 3000))
    # cv2.imshow('Warped', cnt_img)
    # cv2.imshow('Original', smallColr)

    # Added finding contour work:
    # This assumes that the robot is already on the line
    # create array containing closest contours to robot

    # duplicate contours list and remove any contours that are too short
    contours_shorten = list(contours)

    minLength = 19
    # remove any small contours (typically errors)
    for cont in range(len(contours)):
      
        if len(contours[cont]) < minLength:
            contours_shorten.remove(contours[cont])

    contour_pertr = []  # perspective correction contours
    contours_pertrInts = []

    # iterate through contour list
    for cont3 in range(len(contours_shorten)):
        contours_shorten[cont3] = contours_shorten[cont3].reshape(-1, 2)
        # take each contour in term
        a = contours_shorten[cont3]
        a = np.array([a])
        a = a.astype(float)
        # apply perspective transform to each contour
        contour_pertr.append(cv2.perspectiveTransform(a, h))
        contour_pertr[cont3] = np.reshape(np.ravel(contour_pertr[cont3]), (-1, 2))
        contours_pertrInts.append(cv2.perspectiveTransform(a, h))
        contours_pertrInts[cont3] = np.reshape(np.ravel(contours_pertrInts[cont3]), (-1, 2))
        # contours_pertrInts[cont3] = contours_pertrInts[cont3].astype(int)
        # for ind in range(2):
        #     contours_pertrInts[cont3][cont_pertr][ind] = int(contour_pertr[cont3][cont_pertr][ind])
        # contours_pertrInts[cont3] = np.reshape(np.ravel(contours_pertrInts[cont3]), (-1, 1, 2))
        # contours_pertrInts[cont3].astype(int)
        # contour_pertr[cont3] = np.reshape(np.ravel(contour_pertr[cont3]), (-1, 1, 2))
        # contours_shorten[cont3] = a.reshape(-1, 1, 2)
    # contours = contours3
    # contour_pertr.astype(int)

    # draw contours to image
    # cnt_img2 = cv2.drawContours(cnt_img, contours_pertrInts, -1, (0, 255, 0), 3)
    # cv2.imshow('Contours', cnt_img2)

    closestContours = []
    for i in range(len(contours)):  # iterate through each contour
        for point in range(len(contours[i])):  # iterate through the points in each contour
            # find contours with points in middle 165-235 pixels x and bottom 280-300 y - i.e. closest contours
            # thresholds can be changed to ensure just 2 contours are found
            if 100 <= contours[i][point][0][0] <= 300 and 280<= contours[i][point][0][1] <= 299:
                closestContours.append(i) 
                break

    midpointx = []
    midpointy = []
    mptheta = []
    clcont = []

    if len(closestContours) == 0:  # Not on line
       
        if len(contours) >= 1:
         
            for k in range(len(contours)):  # iterate through list of contours
                clcont.append(abs(cv2.pointPolygonTest(contours[k], (200, 299), True)))  # calculate the distance between bottom centre of image and contour
            followContour = clcont.index(min(clcont))  # find the contour with the shortest distance from point
            randpoint = [0, 0]
            p = [200, 299]
            while distance.euclidean(p, randpoint) >= 150 or distance.euclidean(p, randpoint) <= 30:  # choose a random point on contour
                randindex = randint(0, len(contours[followContour])-1)
                randpoint = contours[followContour][randindex][0]


            for k in range(-2, 2): # choose 2 points either side of chosen point
                refindex = randindex + k
                if 0 <= refindex < len(contours[followContour]): # as long as additional points are actually on contour - if a point too close to the end is chosen, other points may fall off end of contour
                    midpointx.append(contours[followContour][refindex][0][0])
                    midpointy.append(contours[followContour][refindex][0][1]) # creates a set of 5 points to head towards on line


            # plot midpoints
            midptList = np.array(list(zip(midpointx, midpointy))) # create one list containing x and y midpoints
            midptList = midptList.reshape((-1, 1, 2)) # ensure midptList is in correct format
            midptList = midptList.reshape(-1, 2)
            b = midptList
            b = np.array([b])
            b = b.astype(float) # convert to float
            midptList_dist = cv2.perspectiveTransform(b, h) # apply perspective transform to midpoints
            midptList_dist = midptList_dist.reshape((-1, 1, 2)) # reshape output array into original contour array form
            # cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0, 255, 255), 1)
            # cv2.namedWindow('midpoints')
            # cv2.imshow('midpoints', cnt_img2)

            for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint of the transformed midpoints (i.e. actual angles)
                mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]), (midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
            mptheta.append(mptheta[len(mptheta)-1]) # add last midpoint angle on at end, this makes angle array same size as midpoint array

       # else:
       # no contours or lines in image
       # Go to Nessa's code to move  robot to locate new contour

    if len(closestContours)==1: # Can only see one contour, choose points on this line to follow
      
        randpoint = [200, 150]
        p = [0, 0]
        while distance.euclidean(p, randpoint) >= 250 or distance.euclidean(p, randpoint) <= 3: # choose random point on contour thats not too close or too far
            randindex = randint(0, len(contours[closestContours[0]])-1) # start by choosing random index on closest contour 
            randpoint = contours[closestContours[0]][randindex][0] # and then find the corresponding point and check the distance
    
       
       # we choose just a few points so that if there is only one horizontal contour the robot can see, it doesnt choose points along the line 
       # on both sides of the robot, as we dont want it to try and go in two different directions
        for k in range(-2, 2): # Again choose 2 points either side of chosen point
            refindex = randindex + k
            if 0 <= refindex < len(contours[closestContours[0]]):
                midpointx.append(contours[closestContours[0]][refindex][0][0]) # robot will drive to these points located on the contours
                midpointy.append(contours[closestContours[0]][refindex][0][1])


        # plot midpoints
        midptList = np.array(list(zip(midpointx, midpointy))) # combine x and y midpoint coordinate data into one list
        midptList = midptList.reshape((-1, 1, 2))

        midptList = midptList.reshape(-1, 2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b, h) # apply perspective transform to midpoint data
        midptList_dist = midptList_dist.reshape((-1, 1, 2)) # reshape transformed midpoints back to original shape
        # cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0, 255, 255), 1)
        # cv2.namedWindow('midpoints')
        # cv2.imshow('midpoints', cnt_img2)
   

        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
        # calculate angle between each transformed midpoint - gives real life angle
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]), (midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1])
      
        
    if len(closestContours)==2: # If two contours are close to robot, then track both
     
        # Pick one of the two contours to follow at random
        followContour = random.choice(closestContours) # choose one of two contours to follow at random
        # print(followContour)
        # dist2 = []
        # Find the other contour that is not being followed
        m = []
        contours2 = list(contours) # copy contours list
        contours2.remove(contours2[followContour]) # remove the contour we are following

        for contour in range(len(contours2)): # iterate through contours excluding followed contour
            if len(contours2[contour]) > 19: # for contours longer than 19, create a combined list of every other contour points
                  contourpoints = np.reshape(np.ravel(contours2[contour]), (-1, 2)).tolist()
                  m = m + contourpoints
        # followContourListPosition = closestContours.index(followContour)
        # otherContour = closestContours[abs(followContourListPosition-1)]
        # otherContourList = np.reshape(np.ravel(contours[otherContour]), (-1, 2))


        # For each point in the followed contour, find the closest point in the other contour
        tree = spatial.KDTree(m) # create tree of all contour points
        for pt in range(0, len(contours[followContour]), 1): # iterate through follow contour
            p = contours[followContour][pt][0] # for each point on the follow contour
            pointonLine = m[tree.query(p)[1]] # find the closest point on the other contour
            lineDistance = distance.euclidean(p, pointonLine) # find the distance between the two lines
            if 5 <= lineDistance <= 130: # if distance between lines is around tape width - i.e. lines not diverging (may need to change)
                midpointx.append((p[0]+pointonLine[0])/2) # add midpoints
                midpointy.append((p[1]+pointonLine[1])/2)

        # plot midpoints
        midptList = np.array(list(zip(midpointx, midpointy))) # combine midpoint coordinates into one list
        midptList = midptList.reshape((-1, 1, 2)) # reshape this new list into the original shape

        midptList = midptList.reshape(-1, 2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b, h) # find perspecitve transform of the midpoints i.e. map them to real life positions
        midptList_dist = midptList_dist.reshape((-1, 1, 2))
        # cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0, 255, 255), 1)

        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
        # iterate through corrected midpoint list, and calculate angle between each midpoint - as midpoints have been persp.corrected, these angles will be real life angle
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]), (midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1]) # add on last angle to list to make angle list same length as midpoint list

        # cv2.namedWindow('midpoints')
        # cv2.imshow('midpoints', cnt_img2)
        # cv2.imwrite('midpts.jpg', cnt_img2)
    
    
    if len(closestContours)==3: # 3 contours found in radius i.e. t junction
  
        dist3 = []
        randcont = []
    # ** Old Code **
    #    for cont3 in range(len(closestContours)):
    #        dist3.append(abs(cv2.pointPolygonTest(contours[closestContours[cont3]], (200, 299), True)))
    #    max_dist_index = dist3.index(max(dist3))
    #    furthestcontour = closestContours[max_dist_index]
    #    randcont.append(furthestcontour) # always follow new contour (starts furthest away)
    #    closestContours.remove(furthestcontour)
    #    randcont.append(random.choice(closestContours))
    #    followContour = random.choice(randcont) # choose at random one of other two contours to follow
    #    followContourListPosition = randcont.index(followContour)
    #    otherContour = randcont[abs(followContourListPosition-1)] # choose at random which contour will be the main contour to follow
    #    otherContourList = np.reshape(np.ravel(contours[otherContour]), (-1, 2))
    
        # **New Code **
        correctcontour = 0
        while correctcontour == 0:
            followContour = random.choice(closestContours) # choose random contour to follow
            # print(followContour)
           
            
            # Find the other contour that is not being followed
            for cnt3 in range(len(contours)): # iterate through contour list
                dist3 = []
                if cnt3 != followContour: # don't choose the contour that is being followed
                    for c in range(0, len(contours[followContour]), round(len(contours[followContour])/10)): # choose 10 points along contour
                        fcPt = contours[followContour][c][0] # points on follow contour
                        dist3.append(abs(cv2.pointPolygonTest(contours[cnt3], tuple(fcPt.tolist()), True))) # find distance between follow contour and these 10 points
                    if all(10 <= dst <=130 for dst in dist3): # if all distances are within a set distance, assume the contours are parallel
                        cnt3list = np.reshape(np.ravel(contours[cnt3]), (-1, 2))
                        tree = spatial.KDTree(cnt3list)
                        pointonLine = cnt3list[tree.query(fcPt)[1]] # find nearest point on other contour
                        midx = int(round((fcPt[0]+pointonLine[0])/2))
                        midy = int(round((fcPt[1]+pointonLine[1])/2))
                        midpt = [midx, midy]
                        if resized[midy][midx] < 100: # check that this midpoint is on a black line, and not the white gap between two lines
                            otherContour = cnt3
                            correctcontour = 1 # set flag to break from while loop
                            break



        otherContourList = np.reshape(np.ravel(contours[otherContour]), (-1, 2))

        # For each point in the followed contour, find the closest point in the other contour

        tree = spatial.KDTree(otherContourList) # create tree of points on other contour
        for pt in range(len(contours[followContour])): # iterate through contour followed
            p = contours[followContour][pt][0]
            pointonLine = otherContourList[tree.query(p)[1]] # find closest point on other contour to each point on follow contour
            lineDistance = distance.euclidean(p, pointonLine)
            if 5 <= lineDistance <= 150: # if distance between lines is around tape width - i.e. lines not diverging (may need to change)
                midpointx.append((p[0]+pointonLine[0])/2) # add midpoints
                midpointy.append((p[1]+pointonLine[1])/2)       



        # plot midpoints
        midptList = np.array(list(zip(midpointx, midpointy))) # join x and y midpoints into one list

        midptList = midptList.reshape(-1, 2)
        b = midptList
        b = np.array([b])
        b = b.astype(float)
        midptList_dist = cv2.perspectiveTransform(b, h) # apply perspective transforms to midpoints
        midptList_dist = midptList_dist.reshape((-1, 1, 2))
        #cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0, 255, 255), 1)

        for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
        # find angle between each midpoint
            mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]), (midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
        mptheta.append(mptheta[len(mptheta)-1])

        #cv2.namedWindow('midpoints')
        # cv2.imshow('midpoints', cnt_img2)
        #cv2.imwrite('midpointsnew',)



    if len(closestContours) >3:
   
        for k  in range(len(contours)): # iterate through contours
            clcont.append(abs(cv2.pointPolygonTest(contours[k], (200, 299), True))) # create array of distances between robot and each contour
            followContour = clcont.index(min(clcont)) # choose the closest contour to follow
            randpoint = [0, 0]
            p =[200, 299]
            while distance.euclidean(p, randpoint) >= 250 or distance.euclidean(p, randpoint) <= 30:
                randindex = randint(0, len(contours[followContour])-1)
                randpoint = contours[followContour][randindex][0]


            for k in range(-2, 2): # choose some points on the closest contour to follow
                refindex = randindex + k
                if 0 <= refindex < len(contours[followContour]):
                    midpointx.append(contours[followContour][refindex][0][0])
                    midpointy.append(contours[followContour][refindex][0][1])


             # plot midpoints
            midptList = np.array(list(zip(midpointx, midpointy)))
            midptList = midptList.reshape((-1, 1, 2))
            midptList = midptList.reshape(-1, 2)
            b = midptList
            b = np.array([b])
            b = b.astype(float)
            midptList_dist = cv2.perspectiveTransform(b, h)
            midptList_dist = midptList_dist.reshape((-1, 1, 2))
            # cv2.polylines(cnt_img2, np.int32([midptList_dist]), True, (0, 255, 255), 1)
            # cv2.namedWindow('midpoints')
            # cv2.imshow('midpoints', cnt_img2)

            for midpt1 in range(len(midptList_dist)-1): # create array of angles between each midpoint
                mptheta.append(np.arctan2((midptList_dist[midpt1+1][0][1]-midptList_dist[midpt1][0][1]), (midptList_dist[midpt1+1][0][0]-midptList_dist[midpt1][0][0])))
            mptheta.append(mptheta[len(mptheta)-1])
    

#    end_time = time.time()-start_time
#    print('time taken: %s'%end_time)
    return [midptList_dist, mptheta] # return list of x and y midpoints and angle list


##test:
#img = cv2.imread('test4.jpg',0)
#[midptList_dist, mptheta] = imgProcess(img)