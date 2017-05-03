# -*- coding: utf-8 -*-
"""
Created on Tue May  2 13:06:10 2017

@author: William
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize

def posCalc(V, pos):
    Vr = V[0]
    Vl = V[1]
    dt = V[2]
    l = 1
    
    if Vr == Vl:
        xNew = pos[0] + (Vr * dt * np.cos(pos[2]))
        yNew = pos[1] + (Vr * dt * np.sin(pos[2]))
        posNew = np.array([xNew, yNew, pos[2]])
    else:
        R = (l/2) * (Vl + Vr) / (Vr - Vl)
        w = (Vr - Vl) /l
        a = w * dt
        ICC = np.array([pos[0] - R * np.sin(pos[2]),
                        pos[1] + R * np.cos(pos[2]),
                        a                         ])

        transfmat = np.array([[np.cos(a), -np.sin(a), 0],
                              [np.sin(a),  np.cos(a), 0],
                              [0,          0,         1]])
        
        temp = np.array([pos[0]-ICC[0],
                         pos[1]-ICC[1],
                         pos[2]])

        posNew = np.dot(transfmat,np.transpose(temp)) + ICC
                           
    
    if posNew[2] > 2*np.pi:
        #Case theta out of range
        while posNew[2] > 2*np.pi:
                posNew[2] = posNew[2] - 2*np.pi
    if posNew[2] < 0 :
        while posNew[2] < 0:
            posNew[2] = posNew[2] + 2*np.pi

    return posNew

def errorCalc(pathData, startPos, targetPos, weighting, motions):
        
    pathShaped = np.reshape(pathData, (motions, 3))    
    guessPos = startPos
    #print("\nSet of ", motions,  ":\n")    
    for i in range(motions):
        guessPos = posCalc(pathShaped[i,:], guessPos)
        #print(guessPos)
    
    distWeight = np.exp(-(((guessPos[0] - targetPos[0])**2) + ((guessPos[1] - targetPos[1])**2)))
    weighting[2] = distWeight * weighting[2]     
    error = guessPos - targetPos
    
    if error[2] > np.pi:
        error[2] = error[2] - 2*np.pi
    elif error[2] < -np.pi:
        error[2] = error[2] + 2*np.pi

    
    weightedError = error*weighting
    
    #print("Weighted error: ", weightedError)
    
    weightedTime = 0#0.01*distWeight*np.sum(pathShaped[:,2])
    finalError = np.linalg.norm(weightedError)  + weightedTime
#    if finalError < 2:                           
#        print("\nPosition: ", guessPos, "\nTarget position:", targetPos,
#              "\nerror", error, "\nweighted error", weightedError, "\nFinal error", finalError) 
    
    return finalError


'''Code Begins'''

mainStartPos  = [0., 0., 0.]
mainTargetPos = [4., 5., 1.]
mainWeighting = [1., 1., 0.]
mainMotions = 1
mainTolerance = 2
mainPathData  = [1, 2, 1]

v_bounds = (-3, 3)
t_bounds = (0, 1)
allBounds = [v_bounds, v_bounds, t_bounds] * mainMotions

#result = scipy.optimize.minimize(errorCalc, mainPathData, args=(mainStartPos, mainTargetPos,
 #                                mainWeighting, mainMotions), bounds=allBounds)
#result = errorCalc(mainPathData, mainStartPos, mainTargetPos, mainWeighting, mainMotions)
#print(result)
#plotStorage = np.zeros((3, 3))
#'''For viewing heat map'''
#for i in range(3):
#    plotStorage[i] = posCalc(mainPathData, mainStartPos)
#    mainStartPos = plotStorage[i]
#plt.plot(plotStorage[:, 0], plotStorage[:, 1], 'ro')
#print(plotStorage)
#vel = np.zeros([mainMotions*3, 1])
#center = [0,0]
#delta = 0.1
#area = 3
#extent = [-area, area, -area, area]
#v_r = np.arange(center[0] - area,center[0] + area, delta)
#v_l = np.arange(center[1] - area,center[1] + area, delta)
#
#V_R, V_L = np.meshgrid(v_r,v_l)
#
#gridresult =np.zeros_like(V_L)
#minError = 10
#
#shape = V_L.shape
#for i in range(shape[0]):
#    for j in range(shape[1]):
#        mainPathData[0] = v_r[i]
#        mainPathData[1] = v_l[j]
#        mainPathData[2] = 1
#        gridresult[i,j] = errorCalc(mainPathData, mainStartPos, mainTargetPos, mainWeighting, mainMotions)
#        if minError > gridresult[i, j]:
#            minError = gridresult[i, j]
#            pathVr = v_r[i]
#            pathVl = v_l[j]
#        
#plt.figure()
#CS = plt.imshow(gridresult,cmap='hot', extent=extent, origin='lower')
#plt.show()
#print("min error: ", minError, "Vr", pathVr, "Vl", pathVl)
