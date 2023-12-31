# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# FnTmodel.py (Version 1.0) = torques and forces calculation simplified model for DJI F450 quadrotor
# __________________________________________________________________________________________________________

import numpy as np
import sys

class QuadFT_Simplified:
    
    def __init__(self, kT, kD, l, pwm2omega_rpm_cof):
        self.PWM_max = 2000
        self.PWM_min = 1000

        self.kT = kT
        self.kD = kD
   
        #print("kT={} \n kD={}".format (kT, kD))
        self.Gamma = np.array([[   kT,   kT,  kT,    kT],
                               [    0, l*kT,   0, -l*kT],
                               [-l*kT,    0,l*kT,     0],
                               [  -kD,   kD, -kD,    kD]])
   
    	
        self.invGamma = np.linalg.inv(self.Gamma)
       
        self.pwm2omega_rpm_cof = pwm2omega_rpm_cof  
        self.pwm2omega_rpm_poly = np.poly1d(pwm2omega_rpm_cof)
       
        # assuming pwm2omega_rpm polynomial remains degree 1
        self.omega_rpm2pwm_cof = [  1/self.pwm2omega_rpm_cof[0], 
                      -self.pwm2omega_rpm_cof[1]/self.pwm2omega_rpm_cof[0] ]
        self.omega_rpm2pwm_poly = np.poly1d(self.omega_rpm2pwm_cof)
        
    def omega_rpm2FT(self, omega_rotors): # Given omega_rotors in R+^4, returns forces and torques
        for i in range(omega_rotors.size):
            assert(omega_rotors[i]>=0) # minus omega is not allowed"
       
        FT = np.dot(self.Gamma, omega_rotors*omega_rotors) # FT = [fb_z,taub_x,taub_y,taub_z]
        return( np.array([0, 0, FT[0]]), FT[1:3+1] ) 
            
    def FT2omega_rpm(self, fb, taub): #Given a desired FT, returns required omega rotors  """
        omega_r_square = np.dot(self.invGamma,np.block([fb[2],taub])) 
        assert ((omega_r_square < 0).sum() == 0) #minus omega_rpm_square in FT2omega_rpm()
        return np.sqrt(omega_r_square)
    
    def pwm2omega_rpm_i(self,pwm_i):  #angular velocity [rad/s]
        if pwm_i <= self.PWM_min:
            return 0
        else:
            return self.pwm2omega_rpm_poly(pwm_i)
    
    def pwm2omega_rpm(self,pwm):
         omega = np.array([ self.pwm2omega_rpm_i(pwm[0]), 
                          self.pwm2omega_rpm_i(pwm[1]),
                          self.pwm2omega_rpm_i(pwm[2]), 
                          self.pwm2omega_rpm_i(pwm[3])])
         return omega
    # the inverse of the above, vector form
    def omega_rpm2pwm(self,omega_rpm): #Given vector [rad/s]
        pwm = np.zeros(4)
        pwm[0] = int(self.omega_rpm2pwm_poly(omega_rpm[0]))
        pwm[1] = int(self.omega_rpm2pwm_poly(omega_rpm[1]))
        pwm[2] = int(self.omega_rpm2pwm_poly(omega_rpm[2]))
        pwm[3] = int(self.omega_rpm2pwm_poly(omega_rpm[3]))
        pwm[pwm < self.PWM_min] = self.PWM_min
        pwm[pwm > self.PWM_max] = self.PWM_max
        return pwm
        
    def pwm2FT(self,pwm):
        #Neglecting drag and aerodynamic component 
        for i in range(4):
            if pwm[i]<self.PWM_min:
                pwm[i]=self.PWM_min
            elif pwm[i]>self.PWM_max:
                pwm[i]=self.PWM_max
            pwm[i]=int(pwm[i])
            
        omega_rpm = self.pwm2omega_rpm(pwm)
        fb, taub = self.omega_rpm2FT(omega_rpm)
        
        return fb, taub
    
    def FT2pwm(self,FT):
        tmp = self.invGamma@FT
        tmp[tmp < self.PWM_min] = self.PWM_min
        omega_rpm = np.sqrt(tmp)
        pwm = self.omega_rpm2pwm(omega_rpm)
        #print(pwm)
        return pwm    
