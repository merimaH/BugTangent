import math 
import numpy as np
class Point:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y
    def point2numpy(self):
        p = np.matrix([self.x],[self])
        p[0] = self.x
        p[1] = self.y
        return p
    def __str__(self):
        return "x " + str(self.x)+ " y "+str(self.y)
    def distance(self,point):
        return math.hypot(self.x-point.x,self.y-point.y)