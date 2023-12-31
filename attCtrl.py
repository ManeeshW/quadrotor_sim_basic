# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# attCtrl.py (Version 1.0)  Attitude controller
#___________________________________________________________________________

import numpy as np 
import pid 
import mathEq

class AttController:

    g = 9.80665 #The Standard gravity is 9.80665 m/s^2

    def __init__(self):
    
        self.dt_ctrl_rate = 0.002

        self.pid_rollrate = pid.PID(50, 0, 0, 8*360*np.pi/180,-8*360*np.pi/180, 0.01)
        self.pid_pitchrate = pid.PID(50, 0, 0, 8*360*np.pi/180,-8*360*np.pi/180, 0.01)
        self.pid_yawrate = pid.PID(50, 0, 0, 8*360*np.pi/180, -8*360*np.pi/180, 0.01)
        
        self.dt_ctrl_angle = 0.004
        
        self.pid_pitch = pid.PID(10, 0, 0, 2.5*360*np.pi/180, -2.5*360*np.pi/180, 0.01)
        self.pid_roll = pid.PID(10, 0, 0, 2.5*360*np.pi/180, -2.5*360*np.pi/180, 0.01)
        self.pid_yaw = pid.PID(5, 0, 0, 2.5*360*np.pi/180, -2.5*360**np.pi/180, 0.01)

    def att_rate(self, ref_omegab, measured_omegab, J ):
        
        alpha_ref = np. zeros(3)
        
        alpha_ref[0] = self.pid_rollrate.run(ref_omegab[0]-measured_omegab[0], self.dt_ctrl_rate)
        alpha_ref[0] = self.pid_rollrate.saturate()
        self.pid_rollrate.antiwindup()
        
        alpha_ref[1] = self.pid_pitchrate.run(ref_omegab[1]-measured_omegab[1],self.dt_ctrl_rate)
        alpha_ref[1] = self.pid_pitchrate.saturate()
        self.pid_pitchrate.antiwindup()
        
        alpha_ref[2] = self.pid_yawrate.run(ref_omegab[2]-measured_omegab[2],self.dt_ctrl_rate)
        alpha_ref[2] = self.pid_yawrate.saturate()
        self.pid_yawrate.antiwindup()
        
        tau_ref = J @ alpha_ref + mathEq.skew(measured_omegab)@ J @ measured_omegab

        return tau_ref

    def att_angle(self, ref_rpy, measured_rpy ):
        
        omega_ref = np. zeros(3)
        
        omega_ref[0] = self.pid_roll.run(ref_rpy[0]-measured_rpy[0],self.dt_ctrl_angle)
        omega_ref[0] = self.pid_roll.saturate()
        self.pid_roll.antiwindup()
        
        omega_ref[1] = self.pid_pitch.run(ref_rpy[1]-measured_rpy[1],self.dt_ctrl_angle)
        omega_ref[1] = self.pid_pitch.saturate()
        self.pid_pitch.antiwindup()
        
       # For 2*pi degrees yaw movement, let's combine the discontinuity -pi + pi 
        err_yaw = ref_rpy[2]-measured_rpy[2]
        if (err_yaw > np.pi):
            err_yaw = -(2*np.pi - err_yaw)
        elif (err_yaw < -np.pi):
            err_yaw = 2*np.pi + err_yaw
        
        omega_ref[2] = self.pid_yaw.run(err_yaw,self.dt_ctrl_angle)
        omega_ref[2] = self.pid_yaw.saturate()
        self.pid_yaw.antiwindup()
        
        return omega_ref
