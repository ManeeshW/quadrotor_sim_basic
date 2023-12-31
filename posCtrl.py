# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# posCtrl.py (Version 1.0) Position controller
#___________________________________________________________________________

import numpy as np 
import pid 
import mathEq
import FnTmodel

class PosController:
    g = 9.80665 #The Standard gravity is 9.80665 m/s^2

    def __init__(self):
        
          self.dt_ctrl_pos_v = 0.1  # 10 Hz
          # saturation in the directional  way 
          self.pid_vx = pid.PID(4, 0, 0, 8000, -8000, 0.1)
          self.pid_vy = pid.PID(4, 0, 0, 8000, -8000, 0.1)
          self.pid_vz = pid.PID(3, 0, 0, 8000, -8000, 0.1)

          self.dt_ctrl_pos_p = 0.1  # 10 Hz
          self.pid_x = pid.PID(1, 0, 0, 20, -20, 0.1)
          self.pid_y = pid.PID(1, 0, 0, 20, -20, 0.1)
          self.pid_z = pid.PID(40, 0, 100, 10, -10, 0.1)
          self.FT = FnTmodel.QuadFT(0)
          self.max_thrust =  self.FT.Trust_max #0.8*0.638  #24 best
    
    def vel(self, ref_ve, measured_ve, measured_yaw, mass):
        
        rp_ref = np.zeros(2)
        
        T1 = self.pid_vx.run(ref_ve[0]-measured_ve[0],self.dt_ctrl_pos_v)
        T2 = self.pid_vy.run(ref_ve[1]-measured_ve[1],self.dt_ctrl_pos_v)
        
        R = np.array([[np.sin(measured_yaw), -np.cos(measured_yaw)],
                              [np.cos(measured_yaw), np.sin(measured_yaw)]])
        
        rp_ref = 1/self.g *R@np.array([T1,T2])
        
        # And saturate 
        V = 30 * np.pi /180 
        v_max = np.max(abs(rp_ref))
        if (v_max > V):
            rp_ref = (V/v_max)*rp_ref
        
        # recalculate to do antiwindup 
        T12 = np.transpose(R) * self.g * rp_ref
        self.pid_vx.u = T12[0]
        self.pid_vx.antiwindup()
        self.pid_vy.u = T12[1]
        self.pid_vy.antiwindup()
        
        T3 = self.pid_vz.run(ref_ve[2] - measured_ve[2], self.dt_ctrl_pos_v)
        thrust_ref = mass*self.g + mass*T3 
 
       # And saturate 
        if ( thrust_ref > self.max_thrust*6 ):
            thrust_ref = self.max_thrust*6
        elif (thrust_ref < 0.2*mass*self.g ):
            thrust_ref =  0.2*mass*self.g 
        
        # recalculate to do anti-windup
        T3 = (thrust_ref - mass*self.g)/mass
        self.pid_vz.u = T3
        self.pid_vz.antiwindup()
        
        return rp_ref, thrust_ref
    
    def pos(self, ref_pos, measured_pos):
        
        ref_ve = np.zeros(3)
        
        ref_ve[0] = self.pid_x.run(ref_pos[0] - measured_pos[0], self.dt_ctrl_pos_p)
        ref_ve[1] = self.pid_y.run(ref_pos[1] - measured_pos[1], self.dt_ctrl_pos_p)
        ref_ve[2] = self.pid_z.run(ref_pos[2] - measured_pos[2], self.dt_ctrl_pos_p)
        
        return ref_ve
