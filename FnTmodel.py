# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# FnTmodel.py (Version 1.0) = torques and forces calculation model for DJI F450 quadrotor 
# __________________________________________________________________________________________________________
#   Model for DJI F450 quadrotor

#   About F450 : https://www.radiolink.com/f450
#
# --DJI F450 quadrotor specifications--
#
#   Size : 450mm (17.72) rotor to rotor 
#        : 225mm  from center of mass to rotor 
#   
#   Weight : 996g (2.2lb With battery)
#   Takeoff Weight Recommended : 350 g/rotor @ 3S LiPo
#                              : 400 g/rotor @ 4S LiPo
#   Takeoff Weight for quadrotor  : 1.4 kg  @ 3S LiPo
#                                 : 1.6 kg  @ 4S LiPo
#                   
#   Brushless Motor : DJI E310 2312 960kV (rpm/V) Motor : https://www.dji.com/e310/spec
#   propeller       : 9450 Propeller (Diameter x Pitch: 9 x 5" / 24 x 13 cm)
#   Max Thrust      : 800 g/rotor @ 12V Sea Level (0.8kg)
#   Max rpm         : 960 rpm/V x 12V = 11520
#   ESC             : Traditional PWM signal (1ms - 2ms) | (1000us - 2000us) is used
#   Moment of inertia [Ix,Iy,Iz] = [0.0035,0.0035, 0.005] kgm^2
#   Drag coefficiant = 1.6 x 10^(-7)
#   
#   To calculate Thrust fcator kT and drag factor kd, we assumed thrust, torque, propeller angular speed
#   into pwm is linear and linear approximation is used. 
#____________________________________________________________________________________________________________

import numpy as np
import sys
import CalCoeff

class QuadFT:
    def __init__(self,x):

        self.l = 0.225 # (m) from center of mass to rotor 
        self.mass = 0.996 # kg
        #PWM 
        self.PWM_max = 2000
        self.PWM_min = 1000
        self.Trust_max = 8
        self.Trust_min = 0.055
        self.Trust2TorqueRatio = 0.1
        self.Omega_rpm_max = 11520
        self.Omega_rpm_min = 300
        self.CofObj = CalCoeff.CalcCoeff(self.Trust_max,self.Trust_min,self.Trust2TorqueRatio,self.Omega_rpm_max,
self.Omega_rpm_min)
        self.Cof = self.CofObj.computeCof()
        self.kT = self.Cof[0]
        self.kD = self.Cof[1]

        self.Kaero = np.ones(3) * 1.6 *10**-7

        #linear approximation
        self.pwm2thrust_cof = self.linearApproximation(self.Trust_max,self.Trust_min,self.PWM_max,self.PWM_min)
        self.thrust2torque_cof = [+0.05, 0] #Trust to torque linear approx.
        self.pwm2omega_rpm_cof = self.linearApproximation(self.Omega_rpm_max,self.Omega_rpm_min,self.PWM_max,self.PWM_min)

        # Matrix of inertia for plus configuration (kgm2)
        self.I = np.array([[3.5,   0, 0],
                           [  0, 3.5, 0],
                           [  0,   0, 5]])*10**-3 
        #polynomial functions    
        self.thrust2torque_poly = np.poly1d(self.thrust2torque_cof)
        self.pwm2thrust_poly = np.poly1d(self.pwm2thrust_cof)
        self.pwm2omega_rpm_poly = np.poly1d(self.pwm2omega_rpm_cof)
        
        # assuming pwm2omega_rpm polynomial remains degree 1
        self.omega_rpm2pwm_cof = [  1/self.pwm2omega_rpm_cof[0], 
                      -self.pwm2omega_rpm_cof[1]/self.pwm2omega_rpm_cof[0] ]
        self.omega_rpm2pwm_poly = np.poly1d(self.omega_rpm2pwm_cof)
		
	#Linear appproximation
    def linearApproximation(self, y_max, y_min, P_max, P_min):
            m = (y_max - y_min)/(P_max - P_min)  
            c = y_max - P_max * m
            cof = [m, c]
            return cof	
    
    def pwm2thrust_i(self, pwm_i): #pwm to rotor thrust [N]
        if pwm_i < self.PWM_min:
            return 0
        return self.pwm2thrust_poly(pwm_i)
    
    def thrust2torque_i(self, ft_i): #thruss to torque [Nm]
        if ft_i == 0:
           return 0
        return self.thrust2torque_poly(ft_i)
    
    def pwm2omega_rpm_i(self, pwm_i): #angular velocity [rad/s]
        if pwm_i <= self.PWM_min:
            return 0
        else:
            return self.pwm2omega_rpm_poly(pwm_i)
    
    # the inverse of the above, vector form
    def omega_rpm2pwm(self, omega_rpm): #angular velocities [rad/s] 
        pwm = np.zeros(4)
        pwm[0] = self.omega_rpm2pwm_poly(omega_rpm[0])
        pwm[1] = self.omega_rpm2pwm_poly(omega_rpm[1])
        pwm[2] = self.omega_rpm2pwm_poly(omega_rpm[2])
        pwm[3] = self.omega_rpm2pwm_poly(omega_rpm[3])
        pwm[pwm < self.PWM_min] = self.PWM_min
        pwm[pwm > self.PWM_max] = self.PWM_max
        return pwm 
    
    def f_aero(self, omega_rpm, vb):
      return np.dot(self.Kaero,vb)*np.sum(omega_rpm)

    def pwm2thrust(self,pwm):
        # thrust on each rotor
        f1 = self.pwm2thrust_i(pwm[0])
        f2 = self.pwm2thrust_i(pwm[1])
        f3 = self.pwm2thrust_i(pwm[2])
        f4 = self.pwm2thrust_i(pwm[3])
        
        return (f1 + f2 + f3 + f4)
    
    def pwm2omega_rpm(self,pwm):
        return np.array([ self.pwm2omega_rpm_i(pwm[0]), 
                          self.pwm2omega_rpm_i(pwm[1]),
                          self.pwm2omega_rpm_i(pwm[2]), 
                          self.pwm2omega_rpm_i(pwm[3]) ])
    
    # "Main" function    
    def pwm2ftau(self, pwm, vb):
        for i in range(4):
            if pwm[i]<self.PWM_min:
                pwm[i]=self.PWM_min
            elif pwm[i]>self.PWM_max:
                pwm[i]=self.PWM_max
            pwm[i]=int(pwm[i])
            
        # thrust on each rotor
        f1 = self.pwm2thrust_i(pwm[0])
        f2 = self.pwm2thrust_i(pwm[1])
        f3 = self.pwm2thrust_i(pwm[2])
        f4 = self.pwm2thrust_i(pwm[3])
            
        # angular velocity on each rotor
        omega_rpm = self.pwm2omega_rpm(pwm)
          
        # aerodynamic force
        fba = self.f_aero(omega_rpm,vb)
         
        # total force
        fb = ( np.array([0,0,f1+f2+f3+f4])+ fba) 
        
        taur1 = self.thrust2torque_i(f1)
        taur2 = self.thrust2torque_i(f2)
        taur3 = self.thrust2torque_i(f3)
        taur4 = self.thrust2torque_i(f4)
            

        # total torques   		
        taub_x = (f2 - f4)*self.l   # rolling moment 
        taub_y = (f3 - f1)*self.l   # pitching moment 
        
        taub_z = -taur1 - taur3 + taur2 + taur4 # yawing moment 
		
        return (fb, np.array([taub_x,taub_y,taub_z]))

