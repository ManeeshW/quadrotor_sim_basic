# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation 
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# plot.py (Version 1.0)  Visualizing the quadrotor using Matplotlib 
#___________________________________________________________________________

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
#from matplotlib.transforms import Affine2D
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
import math
import os 
import sys

class plot:  
    def __init__(self, l = 0.05, propR = 0.03, scale = 2, faxis = True, axisL = 0.1, propSpeed = True):
        self.l = l #length from center of mass to the propeller center (Radius of the quadrotor)
        self.propR = propR #propeller Radius
        self.scale = scale #Scale of the quadrotor visualization
        self.faxis = faxis   #quadrotor body frame is on
        self.axisL = axisL #quadrotor body frame axis length
        self.propSpeed = propSpeed #Visualization of quadrotor propeller speed
        # Attaching 3D axis to the figure
        #plt.style.use('seaborn-whitegrid')
        self.fig = plt.figure(figsize = [9.4, 4.8],dpi = 120)
        self.ax1 = self.fig.add_subplot(6, 6, 1)
        self.ax2 = self.fig.add_subplot(6, 6, 13)
        self.ax3 = self.fig.add_subplot(6, 6, 25)
        self.ax4 = self.fig.add_subplot(6, 6, 6)
        self.ax5 = self.fig.add_subplot(6, 6, 18)
        self.ax6 = self.fig.add_subplot(6, 6, 30)
        self.ax = self.fig.add_subplot(1, 1, 1,projection="3d")
        #self.ax = plt.gca()
        #self.ax.cla()
        self.Po = np.array([0,0,0])
        self.R =  np.identity(3)
        self.P_ref = np.array([0,0,2,0]) #[x,y,z,yaw]
        
        

    def plotlimits(self,x,y,z,axis_Fixed = False):
        self.ax.set_xlim3d([-x + self.Po[0], x + self.Po[0]])
        self.ax.set_ylim3d([-y + self.Po[1], y + self.Po[1]])
        self.ax.set_zlim3d([-z + self.Po[2], z + self.Po[2]])

        if (axis_Fixed == True):
            self.ax.set_xlim3d([-x, x])
            self.ax.set_ylim3d([-y, y])
            self.ax.set_zlim3d([0.0, z])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        
        
        self.ax.set_title('Quadrotor Simulation V1.0')
    def lineb(self,Wo,R):
        W = Wo + np.array([0,0,0.05]) @ R
        plt.plot([0,2], [0,2],[0,3], color='blue',linewidth=0.6,antialiased=False)
    def quadrotorVisT(self):
        l = self.l * self.scale
        
        P = np.array([[l,0,0],[-l,0,0],[0,l,0],[0,-l,0]])
        Wo = np.array([self.Po,self.Po,self.Po,self.Po])
        W = Wo + P@self.R.T
        plt.plot(W[0:2,0], W[0:2,1],W[0:2,2], color='red',antialiased=False,linewidth=0.8,alpha = 1)
        plt.plot(W[2:4,0], W[2:4,1],W[2:4,2], color='blue',linewidth=0.8,antialiased=False, alpha = 1)
        if self.faxis == True:
            self.plotFrame()
        rpm = [0.01,0.01,0.02,0.029]
        if self.propSpeed == True:
           self.propSpeedVis(rpm, x,y,z)
       
        #p = Circle((0, 0), 0.1 ,color='red')
        #self.ax.add_patch(p)
        #art3d.pathpatch_2d_to_3d(p, z=0.3,zdir='z')

        #Draw a line
        #line= art3d.Line3D([2,3,4,5,1,0,8],[2,3,4,5,6,7,8],[3,5,3,5,3,5,3], color='c')
        #self.ax.add_line(line)

        #Draw a polygon
        #xp = [4,6,6,4]
        #yp = [4,6,6,4]
        #zp = [6,8,6,8]
        #poly = list(zip(xp,yp,zp))
        #self.ax.add_collection3d(art3d.Poly3DCollection([poly],color='m', alpha = 0.1))

        #self.propCircle(x+l,y,z)
        #self.propCircle(x,y+l,z)
        #self.propCircle(x-l,y,z)
        #self.propCircle(x,y-l,z)
        
        #self.propCircle(x+l,y,z,0.029,'gray',True,linewidth = 1, alpha = 0.3)
        #self.propCircle(x,y+l,z,0.029,'gray',True,linewidth = 1, alpha = 0.3) 
        #self.propCircle(x-l,y,z,0.029,'gray',True,linewidth = 1, alpha = 0.3)
        #self.propCircle(x,y-l,z,0.029,'gray',True,linewidth = 1, alpha = 0.3)

        #plt.plot([x+l,x-l], [y,y], [z,z],color='red',antialiased=False,linewidth=0.8,alpha = 0.5)
        #plt.plot([x,x], [y+l,y-l], [z,z],color='blue',linewidth=0.8,antialiased=False, alpha = 0.5)
        
        
        #plt.show()

    def propCircle(self,x,y,z,radius = 0.03,color = 'black', fill=False, linewidth = 1, alpha = 1):
        p = Circle((x, y), radius,color = color,fill = fill,linewidth=linewidth,alpha=alpha)
        self.ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=z, zdir=(1,1,1))

    def propSpeedVis(self, rpm, x,y,z):
        l = self.l * self.scale
        R = rpm
        self.propCircle(x+l,y,z,R[0],'red',True,linewidth = 1, alpha = 0.4)
        self.propCircle(x,y+l,z,R[1],'royalblue',True,linewidth = 1, alpha = 0.4)
        self.propCircle(x-l,y,z,R[2],'red',True,linewidth = 1, alpha = 0.4)
        self.propCircle(x,y-l,z,R[3],'royalblue',True,linewidth = 1, alpha = 0.4)

    def plotFrame(self):
        l_axis = self.axisL * self.scale
        P = np.array([[0,0,0],[l_axis,0,0],[0,0,0],[0,l_axis,0],[0,0,0],[0,0,l_axis]])
        Wo = np.array([self.Po,self.Po,self.Po,self.Po,self.Po,self.Po])
        W = Wo + P@self.R.T
        plt.plot(W[0:2,0], W[0:2,1],W[0:2,2], color='red',linewidth=0.2,antialiased=False)
        plt.plot(W[2:4,0], W[2:4,1],W[2:4,2], color='green',linewidth=0.2,antialiased=False)
        plt.plot(W[4:6,0], W[4:6,1],W[4:6,2], color='blue',linewidth=0.2,antialiased=False)
    
    def altitudeIncrease(self,step):
        self.P_ref = self.P_ref + np.array([0,0,step,0])
    def altitudeDecreases(self,step):
        self.P_ref = self.P_ref - np.array([0,0,step,0])
    def fwdIncrease(self,step):
        self.P_ref = self.P_ref + np.array([0,step,0,0])
    def fwdDecrease(self,step):
        self.P_ref = self.P_ref - np.array([0,step,0,0])
    def rightIncrease(self,step):
        self.P_ref = self.P_ref + np.array([step,0,0,0])
    def leftDecrease(self,step):
        self.P_ref = self.P_ref - np.array([step,0,0,0])
    def yawClock(self,step):
        self.P_ref = self.P_ref + np.array([0,0,0,step*np.pi/180])
    def yawCountClock(self,step):
        self.P_ref = self.P_ref - np.array([0,0,0,step*np.pi/180])
    def goBacktoInitialPos(self,x):
        self.P_ref = np.array([0,0,2,0])

