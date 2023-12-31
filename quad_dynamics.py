# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# quaddynamics.py (Version 1.0) 
# Implementation of the Quadrotor kinematics and dynemics using quaternions
#___________________________________________________________________________

import numpy as np
import mathEq 

class dynamics:
    g = 9.80665 #The Standard gravity is 9.80665 m/s^2

    def __init__(self,pos,q,ve,omegab,mass,I):
        self.mass = mass  #kg mass
        self.I = I
        self.pos = pos #Position vector in earth frame
        self.ve = ve #velocity vector in earth frame m/s  

        self.q = q # [s v] quaternion 
        
        self.rotmb2e = mathEq.quat2rotm(q) #the rotation matrix
        self.rpy = mathEq.rotm2exyz(mathEq.quat2rotm(self.q)) #roll pitch yaw
        
        self.vb = np.transpose(self.rotmb2e)@self.ve #velocity in body frame m/s  
        self.omegab = omegab #angular velocity in body frame rad/s

        self.invI = np.linalg.inv(self.I) 
    
        self.dpos = np.zeros(3)
        self.dq = np.zeros(4)
        self.dve = np.zeros(3)
        self.domegab = np.zeros(3)
        
    def dynkin_quadrotor(self,dt,fb,taub): #Dynamics and Kinematics for the quadrotor motion
    
        self.dpos = self.ve # dpos/dt = ve 
        
        # dve/dt = 1/m*Rbe*fb + g
        self.dve = 1/self.mass*mathEq.quat2rotm(self.q) @ fb + np.array([0,0,-self.g])
        # dq/dt
        self.dq=np.array([(-self.q[1]*self.omegab[0]-self.q[2]*self.omegab[1]-self.q[3]*self.omegab[2]),
                          (self.q[0]*self.omegab[0]-self.q[3]*self.omegab[1]+self.q[2]*self.omegab[2]),
                          (self.q[3]*self.omegab[0]+self.q[0]*self.omegab[1]-self.q[1]*self.omegab[2]),
                          (-self.q[2]*self.omegab[0]+self.q[1]*self.omegab[1]+self.q[0]*self.omegab[2])])*0.5
        
        # domegab/dt = I^(-1)*(-skew(omegab)*I*omegabb + taub)
        self.domegab = (np.dot(self.invI,np.dot(-mathEq.skew(self.omegab),np.dot(self.I,self.omegab))+ taub))
    
        
        x = np.concatenate([self.pos, self.q, self.ve, self.omegab]) #state vector x
        dx = np.concatenate([ self.dpos, self.dq, self.dve, self.domegab ])
        # Integrate for over small step dt
        x = x + dt*dx
  
        ###### From State Vector #########
        self.pos = x[0:3]
        self.q = x[3:7] / np.linalg.norm(x[3:7])  #update and normalize 
        self.rotmb2e = mathEq.quat2rotm(self.q)
        self.rpy = mathEq.rotm2exyz(self.rotmb2e)
        self.ve = x[7:10]
        self.vb =np.transpose(self.rotmb2e)@self.ve
        self.omegab = x[10:13]
        
            
