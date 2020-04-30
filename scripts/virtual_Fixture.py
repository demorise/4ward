#!/usr/bin/env python

import math 
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 


def virtualPlane(endEffectorPos):
    #Object Generation
    x1 = endEffectorPos[0]
    y1 = endEffectorPos[1]
    z1 = endEffectorPos[2]
    
    a = 1
    b = 1
    c = 1
    d = 0
    
    #Using https://technology.cpm.org/general/3dgraph/ to visualize plane
    #Defining and finding distance to plane from point
    d = abs((a * x1 + b * y1 + c * z1 + d))  
    e = (math.sqrt(a * a + b * b + c * c)) 
    dist = d/e

    normal = [a,b,c]
    normalunit = []
    for i in normal:
        normalunit.append(i/math.sqrt(a**2+b**2+c**2)) #Unit Normal Vector

    #Pushing point away from plane
    newEndEffectorPos = []
    if dist<1:
        for i,name in enumerate(endEffectorPos):
            newEndEffectorPos.append(normalunit[i]*.1)
            newEndEffectorPos[i] = endEffectorPos[i] + newEndEffectorPos[i]
            
        #Checking if Location of next point closer to plane
        d = abs((a * newEndEffectorPos[0] + b * newEndEffectorPos[1] + c * newEndEffectorPos[2] + d))  
        e = (math.sqrt(a * a + b * b + c * c)) 
        dirDist = d/e

        #If Location of next point closer to plane, reverse directions
        if dirDist <= dist:
            for i,name in enumerate(endEffectorPos):
                newEndEffectorPos[i] = -normalunit[i]*.1
                newEndEffectorPos[i] = endEffectorPos[i] + newEndEffectorPos[i]
        # print(newEndEffectorPos)
        return newEndEffectorPos
    else:
        newEndEffectorPos = endEffectorPos
        # print(newEndEffectorPos)
        return newEndEffectorPos
        
    

def main():
    pts = [0,0,0]
    points = []
    for i in range(-5,5):
        for j in range(-5,5):
            for k in range(-5,5):
                pts[0],pts[1],pts[2] = i*.1,j*.1,k*.1
                points.append(list(virtualPlane(pts)))  
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i,n in enumerate(points):
        ax.scatter(points[i][0],points[i][1],points[i][2])
        
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()
    
if __name__ == "__main__":
    main()
    
