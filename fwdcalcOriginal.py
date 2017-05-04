import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt

L = 1
motions = 3 #Do not change this
tolerance = 0.5
def posCalc(V, pos):
'''Function returns new position after single time step
   from current positon, wheel velocities and time step '''
    Vr = V[0]
    Vl = V[1]
    dt = V[2]
    
    if Vr == Vl: #Case wheel velocities same. R equals inf. No rotation
        xNew = pos[0] + (Vr * dt * np.cos(pos[2]))
        yNew = pos[1] + (Vr * dt * np.sin(pos[2]))
        posNew = np.array([xNew, yNew, pos[2]])
    else: #Case wheels diff speeds. Will have rotation may also have translation. 
        R = (L/2) * (Vl + Vr) / (Vr - Vl)
        w = (Vr - Vl) /L
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

        posNew = np.dot(transfmat,temp) + ICC
                               
    if posNew[2] > 2*np.pi:
        #Case theta out of range. Ensures theta between 0-2pi
        while posNew[2] > 2*np.pi:
                posNew[2] = posNew[2] - 2*np.pi
    if posNew[2] < 0 :
        while posNew[2] < 0:
            posNew[2] = posNew[2] + 2*np.pi

    return posNew #Returns new position in form (x, y, theta) 

    

def guessVel(target, coord):
    '''Function returns set of velocities and times in single dimensional array.
       All times are set to 1.
       Function requires single target coordinate, current coordinate and number of steps'''

    tempCoord = np.zeros(2)
    vel = np.ones(3*motions)
    
    #Guess initial value of vel
    #Step 1
    travelTheta = np.arctan2(target[1]-coord[1], target[0]-coord[0])
    #Calculate the angle from current position to target        
    vel[0] = L * (travelTheta - coord[2])/2
    vel[1] = -vel[0]   
    #Set wheel velocity to rotate to point towards target
    
    if travelTheta - coord[2] > np.pi:
        travelTheta = travelTheta - 2*np.pi
        vel[0] = -vel[0]
        vel[1] = -vel[1]
    elif travelTheta < -np.pi:
        travelTheta = travelTheta + 2*np.pi
        vel[0] = -vel[0]
        vel[1] = -vel[1]
        
    #Code ensures robot makes smallest rotation to face target 
            
    #Step 2
    vel[3] = np.sqrt((target[0]-coord[0])**2+(target[1]-coord[1])**2)
    if vel[3] > 3:
              vel[3] = 3
    vel[4] = vel[3]
    
    #Set wheel velocities to move in straight line towards target
    #If required speed is greater than max set to max
    
    tempCoord[0] = coord[0] + vel[3]*np.cos(travelTheta)
    tempCoord[1] = coord[1] + vel[3]*np.sin(travelTheta)
    
    #Update current coordinate
    
    #step 3        
    if ((target[0] - tempCoord[0]) > 0.5) and \
       ((target[1] - tempCoord[1]) > 0.5):
    #If far from targets x, y translate again else rotate
    #Rotate as seen in step 1. Translate as seen in step 2       
                   
        vel[6] = np.sqrt((target[0]-coord[0])**2+(target[1]-coord[1])**2) 
        
        if vel[6] > 3:
            vel[6] = 3
        vel[7] = vel[6]
    else:
        vel[6] = L * (target[2] - travelTheta)/2
        vel[7] = -vel[6]
        
        if (travelTheta - target[2]) > np.pi:
            travelTheta = travelTheta - 2*np.pi
            vel[6] = -vel[6]
            vel[7] = -vel[7]
        elif (travelTheta - target[2]) < -np.pi:
            travelTheta = travelTheta + 2*np.pi
            vel[6] = -vel[6]
            vel[7] = -vel[7]
        
    return vel #Return list of velocities and time in 1D array. 
        
def errorCalc(V, current_coord, target, weighting):
	'''Inputs: Wheel velocity and facing angle [Vr[motions],Vl[motions],dt[motions]], 
       current position [X, Y, theta], desired position [X, Y, theta] and number of time step.
       Return: Error between current position after all time steps and target position,
       with time as a consideration.'''
    vShaped = np.reshape(V, (motions,-1))
    #Shape motor instructions to have single instruction per row 
#    print("\n", vShaped,
#          "\n", current_coord,
#          "\n", target,
#          "\n", weighting, 
#          "\n", motions)
    
    for i in range(motions):
        current_coord = posCalc(vShaped[i], current_coord)
    #calc final position from set of motor instructions  
        
    distWeight = np.exp(-(((current_coord[0] - target[0])**2) + ((current_coord[1] - target[1])**2)))
    weighting[2] = distWeight * weighting[2]
    #Produce weighting with theta weighting dependant on x, y distance from target    
    error = current_coord - target
    #Unweighted error 

    if error[2] > np.pi:
        error[2] = error[2] - 2*np.pi
    elif error[2] < -np.pi:
        error[2] = error[2] + 2*np.pi
    #Ensure theta error in range -pi to pi          
             
    weightedTime = 0.1*np.sum(vShaped[:,2])*distWeight
    weightedError = np.multiply(error, weighting)
    finalError = np.linalg.norm(weightedError)  + weightedTime
    #Final error withweighted x, y, theta considered and weighted total time. Float                           
    return finalError #return error as float

def routeCalculation(targetArray, coord, allbounds, overallSteps):
'''Calculates route by minimising error for each step towards a target. Function loops through
   until all target have been reached. Function returns n by 3 array of motor instructions
   in the form (Vr,Vl, dt). ''' 
    
    pointNo = 0
    stepNo = 0
    target = targetArray[0]
    vToMotor = []        

    while pointNo < overallSteps :
    #Loops while the number of targets reached is less than the total number of targets     
#        print("\nPos before step: ", coord,
#              "\nvel before step: ", vel,
#              "\ntarget before step: ", target,
#              "\nweighting before step", weighting,
#              "\nmotions before step", motions)

        vel = guessVel(target, coord)  
        #Produces an good intital guess for motor instructions. See def guessVel()
        result = scipy.optimize.minimize(errorCalc, vel, args=(coord, target, weighting), bounds=allbounds)
		#Minimise error for moving towards target in 3 (motions) steps	

        vel = result.x
        temp = np.reshape(result.x, (motions, 3))
        #Format returned motor instructions into more useful shape
        
        for i in temp:
            vToMotor.append(i)
        #Add motor instructions to vToMotor as row of three cols    
        for i in range(motions):
            coord = posCalc(temp[i], coord)
        
        stepNo = stepNo + 1 
        #Increase number of steps. Currently does nothing may want a limit to prevent constant loop                                    
        if (result.fun < tolerance) and (pointNo < overallSteps-1):
            pointNo = pointNo + 1
            target = targetArray[pointNo]
            #If target reached and targets left update target
        elif (result.fun < tolerance) and (pointNo == overallSteps -1):
            pointNo = pointNo + 1
            #If target reached and no targets left 
    
    return np.array(vToMotor) #Return array of n rows and 3 cols. (Vr, Vl, dt)

def viewHeatMap(coord, targetArray):
    '''Function prints heatmap for each target assuming a single step of one second is made'''
    vel = np.zeros([motions*3, 1])
    center = [0,0]
    delta = 0.1
    area = 5
    extent = [-area, area, -area, area]
    v_r = np.arange(center[0]-area, center[0]+area, delta)
    v_l = np.arange(center[1]-area, center[1]+area, delta)
    
    V_R, V_L = np.meshgrid(v_r,v_l)
    
    gridresult =np.zeros_like(V_L)
    for n in range(targetArray.shape[0]):
        print(n)
        target = targetArray[n]
        shape = V_L.shape
        for i in range(shape[0]):
            for j in range(shape[1]):
                vel[0] = v_r[i]
                vel[1] = v_l[j]
                vel[2] = 1
                gridresult[i,j] = errorCalc(vel, coord, target, weighting)
                
        plt.figure()
        CS = plt.imshow(gridresult,cmap='hot', extent=extent, origin='lower')
        plt.show()
        
def viewPathPlot(vToMotor, coord):
    '''Function plots path in x, y taken by robot'''
    plt.plot(coord[0],coord[1], 'ro')
    
    
    vToMotor[:, 2] = vToMotor[:, 2]/15
            
    #For viewing plot
    for i in range(len(vToMotor)):             
        for j in range(15):
            plotPos = posCalc(vToMotor[i], coord)
            coord = plotPos
            plt.plot(plotPos[0],plotPos[1], 'ro')
    plt.show()
    
'''Code Begins Here'''
#Variable declaration

targetArray = np.array([[1, 1, 0.7], 
                        [2, 1, 5], 
                        [-1, -2, 0]])
overallSteps = len(targetArray)
                   
coord = np.array([0.0 ,0.0 ,0.0])

v_bounds = (-3, 3)
t_bounds = (0, 1)
allBounds = [v_bounds, v_bounds, t_bounds] * motions
                    
weighting = np.array([1,1,1])
        
        
if __name__ == "__main__":
        
    #print("New")
            
    vToMotor = routeCalculation(targetArray, coord, allBounds, overallSteps)
            
    print("To motor: ", vToMotor)
    #viewHeatMap(coord, targetArray)
    #(vToMotor, coord)
    #print("End")
    








