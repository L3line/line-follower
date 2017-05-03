
import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt

def fwdCalc(V, current_coord, delta_t):
    '''Inputs: Wheel velocity and facing angle [Vr, Vl], current position [X,Y,theta], and time step.
       Return: Estimated position [X, Y, theta] 
    '''
    
#    print("\nPreallocate position: ", current_coord)
    V_r = V[0]
    V_l = V[1]
    x     = current_coord[0][0]
    y     = current_coord[1][0]
    theta = current_coord[2][0]
#    print("\nPosition fwd calc: ", x, y, theta)
    
#    if type(x) is not float:
#            x = x[0]
#            y = y[0]
#            theta = theta[0]
    
    print("\nPosition fwd calc change: ", x, y, theta)
        #Values of variables in assinged for readability    
    l = 0.14 #distance between wheels = l (145 mm)
    
    if V_r == V_l:
        #Case Vr = Vl no rotation so equations of motion different.  
        x_new = x + np.cos(theta) * V_r * delta_t
        y_new = y + np.sin(theta) * V_r * delta_t
        theta_new = theta
        guess_pos = np.matrix([x_new, y_new, theta_new])
    else:         
    #Case with rotation
    #print("Vl", V_l, "\nVr", V_r)
        R= l/2 * (V_l + V_r) / (V_r - V_l)
        w = (V_r - V_l) /l
        a = w * delta_t
        transfmat = np.matrix([[np.cos(a), -np.sin(a), 0],
                               [np.sin(a),  np.cos(a), 0], 
                               [0,          0,         1]]) 
        ICC = np.zeros([3,1])
        ICC[0] = x - R*np.sin(theta)
        ICC[1] = y + R*np.cos(theta)
        offset_pos = current_coord - ICC
        ICC[2] = a
        guess_pos = np.dot(transfmat, offset_pos) + ICC
        
        if guess_pos[2, 0] > 2*np.pi:
            #Case theta out of range
            while guess_pos[2, 0] > 2*np.pi:
                guess_pos[2, 0] = guess_pos[2, 0] - 2*np.pi
        if guess_pos[2, 0] < 0 :
            while guess_pos[2, 0] < 0:
                guess_pos[2, 0] = guess_pos[2, 0] + 2*np.pi
    return guess_pos #Estimated position returned form [X, Y, theta]
        

def errorCalc(V, current_coord, target, weighting, motions):
    '''Inputs: Wheel velocity and facing angle [Vr[motions],Vl[motions],dt[motions]], current position [X, Y, theta], desired position [X, Y, theta] and number of time step.
       Return: Error between current position after all time steps and target position, with time as a consideration.  
    '''
    vShaped = np.reshape(V, (motions,3))
    guessPos = np.zeros([motions, 3])
    
    for i in range(motions):
        a = fwdCalc([vShaped[i, 0], vShaped[i, 1]], current_coord, vShaped[i,2])
        guessPos[i, :] = a.ravel()
        current_coord = np.swapaxes([guessPos[i,:]],0,1)
        
    distWeight = np.exp(-(((current_coord[0] - target[0])**2) + ((current_coord[1] - target[1])**2)))
    weighting[2] = distWeight * weighting[2]     
    error = guessPos[i,:] - target
    

    if error[2] > np.pi:
        error[2] = error[2] - 2*np.pi
    elif error[2] < -np.pi:
        error[2] = error[2] + 2*np.pi 
             
    weightedTime = 0.01*distWeight*np.sum(vShaped[:,2])
    weightedError = np.multiply(error, weighting)
    finalError = np.linalg.norm(weightedError)  + weightedTime
                               
    return finalError

def routeCalculation(vel, targetArray, coord, allbounds, motions, overallSteps, tolerance):
    
    pointNo = 0
    stepNo = 0
    target = targetArray[0]
    vToMotor = []

    while pointNo < overallSteps :
        
        #print("\nPos before step: ", coord,
#              "\nvel before step: ", vel,
#              "\ntarget before step: ", target,
#              "\nweighting before step", weighting,
#              "\nmotions before step", motions)
        
        result = scipy.optimize.minimize(errorCalc, vel, args=(coord, target, weighting, motions), bounds=allbounds)      
        vel = result.x
        temp = np.reshape(result.x, (motions, 3))  
        #print("GuessPos:\n", guessPos)
        
        for i in temp:
            vToMotor.append(i)
            
        #print("\n\ncoord: ", coord)
        print("danger")
        for i in range(motions):
            a = fwdCalc([temp[i, 0], temp[i, 1]], coord, temp[i,2])
            guessPos = a.ravel()
            coord = np.swapaxes(guessPos,0,1)
            
        stepNo = stepNo + 1 
        
        if stepNo > 10:
            print("\nI have failed\n")
            break
                                            
        if (result.fun < tolerance) and (pointNo < overallSteps-1):
            print("Point reached: ", target)
            print("Actual position: ", coord)
            pointNo = pointNo + 1
            target = targetArray[pointNo]
            print("New target: ", target)
        elif (result.fun < tolerance) and (pointNo == overallSteps -1):
            pointNo = pointNo + 1
    
        
        
    print('Final Coord: ', coord)
    return np.array(vToMotor)

'''Code Begins Here'''
#Variable declaration
motions = 3 #Do not change this
tolerance = 1

#for i in range(6):
#    for j in range(6):
targetArray = [[0, 0, 0]]
overallSteps = len(targetArray)
                   
coord = np.array([[0.0],[0.0],[0.0]])
vel = np.ones([motions*3, 1])
vel [1] = 1
v_bounds = (-3, 3)
t_bounds = (0, 1)
allBounds = [v_bounds, v_bounds, t_bounds] * motions
                    
weighting = np.array([[1],[1],[1]])
        
        #guessPos = np.zeros([overallSteps, 3])
        
if __name__ == "__main__":
        
    print("New")
            
    vToMotor = routeCalculation(vel, targetArray, coord, allBounds, motions, overallSteps, tolerance)
            
    print("To motor: ", vToMotor)
    print("End")
    

#coord = [[0], [0], [0]]    
#plt.plot(coord[0],coord[1], 'ro')
##For viewing plot
#for i in range(len(vToMotor)):
##    dt = vToMotor[i, 2]/15             
#    for j in range(15):
#        plotPos = fwdCalc([vToMotor[i, 0], vToMotor[i, 1]], coord, vToMotor[i, 2]/15)
#        coord = plotPos
#        plt.plot(plotPos[0],plotPos[1], 'ro')
#print(coord)
       
#plt.plot(targetArray[0][0], targetArray[0][1], 'bo')
#plt.plot(targetArray[1][0], targetArray[1][1], 'bo')
#plt.plot(targetArray[2][0], targetArray[2][1], 'bo')
#plt.show()

#For viewing heat map
#vel = np.zeros([motions*3, 1])
#result = [0,0]
#delta = 0.1
#area = 5
#extent = [-area, area, -area, area]
#v_r = np.arange(result[0]-area,result[0]+area, delta)
#v_l = np.arange(result[1]-area,result[1]+area, delta)
#
#V_R, V_L = np.meshgrid(v_r,v_l)
#
#gridresult =np.zeros_like(V_L)
#targetArray = [[1,0,0]]
#shape = V_L.shape
#for i in range(shape[0]):
#    for j in range(shape[1]):
#        vel[0] = v_r[i]
#        vel[1] = v_l[j]
#        vel[2] = 1
#        #gridresult[i,j] = errorCalc(vel, coord, targetArray, weighting, motions)
#        
#plt.figure()
#CS = plt.imshow(gridresult,cmap='hot', extent=extent, origin='lower')
#plt.show()

