# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# Quad.py (Version 1.0) =  Quadrotor simulatior main function
#___________________________________________________________________________


#Imported standard libraries
import numpy as np
import time
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#Imported files 
import quad_dynamics
import FnTmodel
import attCtrl
import posCtrl
import mathEq 
import plot
import FnTmodelSim

g = 9.80665 #The Standard gravity is 9.80665 m/s^2

flag = True


######### Initialize the parameters of the quadrotor mathematical model #########################

plus = True #""" Quadrotor configuration, plus or cross """
pos = np.array([0,0,2]) #position vetcor relative to the earth frame
q = np.array([1,0,0,0]) #quaternion (orientation)
ve = np.array([0,0,0])  #Velocity vector relative to the earth frame
omegab = np.array([0,0,0]) #angular velocity vector relative to the body frame 

#Quadrotor Model for the forces and torques 
quadFT = FnTmodel.QuadFT(0) 

#Simplified model for the forces and torques
quadFT_sim = FnTmodelSim.QuadFT_Simplified(quadFT.kT, quadFT.kD, quadFT.l, quadFT.pwm2omega_rpm_cof)

#Quadrotor dynamic model
quad = quad_dynamics.dynamics(pos, q, ve, omegab, quadFT.mass, quadFT.I) 


#Initialize the parameters of the quadrotor visualization model 
L = 0.225     #length from center of mass to the propeller center
propR = 0.12 #propeller Radius (DJI 9450 Propeller) diam = 9.4 inch 24cm
scale = 1   #Scale of the quadrotor visualization
axis = False  #quadrotor body frame is on
axisL = 0.1  #quadrotor body frame axis length
propSpeed = False #Visualization of quadrotor propeller speed
theta = np.pi/4

p = plot.plot(L,propR,scale,axis,axisL,propSpeed)     

# Initialize controller  
##########################################################
att_controller = attCtrl.AttController()
pos_controller = posCtrl.PosController()

omegab_ref = np.zeros(3)
tau_ref = np.zeros(3)
thrust_ref = quad.mass*g
rpy_ref = np.zeros(3)
pos_ref = np.zeros(3)
ref = np.array([0.0,0.0,2.0,0.0]) #  x,y,z,yaw

      
# Simulation parameters

d_simtime = 0.004 #0.0005  
""" integration step """
#dt_log = 0.1
""" logging step """
dt_vis = 1/60   
""" visualization frame step """
t = 0
""" time variable """

i = 0
pt = 0
x = 0

def press(event):
        global flag
        sys.stdout.flush()
        pos_step = 0.01 #1cm
        rot_step = 2 #deg
        if event.key == 'p':
            p.altitudeIncrease(pos_step) 
        if event.key == ';':
            p.altitudeDecreases(pos_step)
        if event.key == "'":
            p.yawClock(rot_step)
        if event.key == 'l':
            p.yawCountClock(rot_step)
        if event.key == 'up':
            p.fwdIncrease(pos_step)
        if event.key == 'down':
            p.fwdDecrease(pos_step)
        if event.key == 'right':
            p.rightIncrease(pos_step)
        if event.key == 'left':
            p.leftDecrease(pos_step)
        if event.key == '0' or event.key == 'o': #[0,0,2]
            p.goBacktoInitialPos(0)
        if event.key == 'q' or event.key == 'escape':
            flag = False
        #print('pressed', event.key)
        print(p.P_ref)

def redraw_figure():
    plt.draw()
    plt.pause(0.01)   

   
def update(n): # main simulatior function
    
    global omegab_ref, tau_ref, thrust_ref, rpy_ref, pos_ref
    global ref, pwm
    global flag # simulatior exit
 
    global i, x
    
    global t, pt

    #+++++++++++++++++++++++ begin controller  ++++++++++++++++++++#

    #Reference input given by the keyboard or the remote controller 
    pos_ref = p.P_ref
    rpy_ref[2] = p.P_ref[3]

    # Assume measurements are perfect  (No noise)
    pos_measured = quad.pos
    ve_measured =  quad.ve
    yaw_measured = quad.rpy[2]
    meas_rpy = quad.rpy
    omegab_measured = quad.omegab
      

    # Attitude controller call 
    omegab_ref = att_controller.att_angle(rpy_ref, meas_rpy)
    tau_ref = att_controller.att_rate(omegab_ref,omegab_measured,quad.I)

    #Position controller call
    ve_ref = pos_controller.pos(pos_ref, pos_measured)
    rp_ref, thrust_ref = pos_controller.vel(ve_ref, ve_measured, yaw_measured, quad.mass)
       
    #Roll Pitch Reference
    rpy_ref[0] = rp_ref[0] 
    rpy_ref[1] = rp_ref[1]
        
    
    # Calculate PWM  using quadFT simplified model 
    pwm = quadFT_sim.FT2pwm(np.array([thrust_ref,tau_ref[0],tau_ref[1],tau_ref[2]]))
   
    #+++++++++++++++++++++++ end controller  +++++++++++++++++++++++#

    # Calculate body-based forces and torques
    fb, taub = quadFT.pwm2ftau(pwm,quad.vb)
    
    # the dynemic model
    quad.dynkin_quadrotor(d_simtime, fb, taub)

    # Time has increased now
    t = t + d_simtime
    st = time.time()
    ct = st - pt 
    pt = st
    #print(ct)
    #print(pwm)

    #++++++++++++++++++++++Quadrotor Visualization++++++++++++++++++++
    if (i%100 == 0):
        p.Po = quad.pos
        p.R = quad.rotmb2e
        p.fig.canvas.mpl_connect('key_press_event', press)
        plt.cla()
        p.plotlimits(0.5,0.5,3,True)
        p.quadrotorVisT()
        redraw_figure()
    i = i+ 1

    #plt.plot([0,0], [0,0],[0,x], color='green',linewidth=0.2,antialiased=False)
    #plt.plot([0,0], [0,0],[0,p.P_ref[2]], color='green',linewidth=0.2,antialiased=False)
    
    return flag 

# Start the visualization
p.plotlimits(0.5,0.5,3)
p.quadrotorVisT()

plt.draw()
plt.pause(0.0001)

         
#ani = animation.FuncAnimation(p.fig, update,interval=0.0001)
#plt.show()

