# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# mathEq.py (Version 1.0) math converter 
#___________________________________________________________________________

import numpy as np

def quat2rotm(q): #convert quaternion into the rotation matrix
    R = np.array([[q[0]**2+q[1]**2-0.5,(q[1]*q[2]-q[0]*q[3]),(q[1]*q[3]+q[0]*q[2])],
                  [(q[1]*q[2]+q[0]*q[3]),(q[0]**2+q[2]**2-0.5),(-q[0]*q[1]+q[2]*q[3])],
                  [(q[1]*q[3]-q[0]*q[2]),(q[2]*q[3]+q[0]*q[1]), q[0]**2+q[3]**2-0.5]])*2 
    return R

def skew(omega): #Returns the skew-symmetric matrix form of the input vector
    omega_hat = np.array([[0,-omega[2],omega[1]],[omega[2],0,-omega[0]],[-omega[1],omega[0],0]]) 
    return omega_hat

def rotm2exyz(R): #Transforms a rotation matrix to Euler X-Y-Z angles
    Set =1 # Choose one set of rotations

    if R[2,2]!=1 and R[2,2]!=-1:
        theta1 = -np.arcsin(R[2,0])
        theta2 = np.pi - theta1
        psi1 = np.arctan2(R[2,1]/np.cos(theta1),R[2,2]/np.cos(theta1))
        psi2 = np.arctan2(R[2,1]/np.cos(theta2),R[2,2]/np.cos(theta2))
        phi1 = np.arctan2(R[1,0]/np.cos(theta1),R[0,0]/np.cos(theta1))
        phi2 = np.arctan2(R[1,0]/np.cos(theta2),R[0,0]/np.cos(theta2))
        
        if Set == 1:
            return np.array([psi1,theta1,phi1])
        else:
            return np.array([psi2,theta2,phi2])     
    else:
        phi = 0 
        if R[2,0] == -1:
            theta = np.pi/2
            psi = phi + np.arctan2(R[0,1],R[0,2])
        else:
            theta = -np.pi/2
            psi = -phi + np.arctan2(-R[0,1],R[0,2])
        return np.array([psi,theta,phi])



