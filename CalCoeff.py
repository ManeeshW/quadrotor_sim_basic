# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# CalCoeff.py (Version 1.0) kT and kD coefficient calculation using linear approximation. 
# These cofficients are need to be determined experimentally to match with real values
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

class CalcCoeff:

    def __init__(self,Trust_max,Trust_min,Trust2TorqueRatio,Omega_rpm_max,Omega_rpm_min,PWM_max = 2000,PWM_min = 1000):
        self.Trust_max = Trust_max
        self.Trust_min = Trust_min
        self.Omega_rpm_max = Omega_rpm_max
        self.Omega_rpm_min = Omega_rpm_min
        self.PWM_max = PWM_max
        self.PWM_min = PWM_min
        self.pwm2thrust_cof = self.linearApproximation(self.Trust_max,self.Trust_min,self.PWM_max,self.PWM_min)
        self.thrust2torque_cof = [+Trust2TorqueRatio, 0] #Trust to torque ratio linear approx.
        self.pwm2omega_rpm_cof = self.linearApproximation(self.Omega_rpm_max,self.Omega_rpm_min,self.PWM_max,self.PWM_min)
        self.pwm2thrust_poly = np.poly1d(self.pwm2thrust_cof)
        self.thrust2torque_poly = np.poly1d(self.thrust2torque_cof)
        self.pwm2omega_rpm_poly = np.poly1d(self.pwm2omega_rpm_cof)


    def linearApproximation(self,y_max,y_min,P_max,P_min):
        m = (y_max - y_min)/(P_max - P_min)  
        c = y_max - (P_max * m)
        coeff = [m, c]
        return coeff

    def pwm2thrust_i(self, pwm_i): #pwm to rotor thrust [N]
        if pwm_i < self.PWM_min:
            return 0
        return self.pwm2thrust_poly(pwm_i)
    
    def thrust2torque_i(self, FT_i): #thruss to torque [Nm]
        if FT_i == 0:
           return 0
        return self.thrust2torque_poly(FT_i)
    
    def pwm2omega_rpm_i(self, pwm_i): #angular velocity [rad/s]
        if pwm_i <= self.PWM_min:
            return 0
        else:
            return self.pwm2omega_rpm_poly(pwm_i)

    def computeCof(self):
        PWM_min = self.PWM_min+1
        thrust = np.zeros(self.PWM_max-PWM_min)
        omega_rpm_sqd = np.zeros(self.PWM_max-PWM_min)
        torque = np.zeros(self.PWM_max-PWM_min)
        x = []
        for i in range(self.PWM_max-PWM_min):
            pwm = PWM_min + i
            thrust[i] = self.pwm2thrust_i(pwm)
            omega_rpm_sqd[i] = self.pwm2omega_rpm_i(pwm)**2
            torque[i] = self.thrust2torque_i(thrust[i])

        kT = np.mean(thrust/omega_rpm_sqd)
        kD = np.mean(torque/omega_rpm_sqd)
        return [kT,kD]

