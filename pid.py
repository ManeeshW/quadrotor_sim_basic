# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (for plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# pid.py (Version 1.0) = Implementation of the discrete time PID controller
# To convert to discrete time, we use the Tustin or trapezoidal rule,
# where the Laplace variable s is replaced with the z-transform approximation
#___________________________________________________________________________
	
class PID:
    #Initialize the PID with the kp, ki and kd constants, saturation limits and tau
    def __init__(self, kp, ki, kd, limitUp, limitDown, tau):
        self.kp = kp #proportional term
        self.ki = ki #integral term
        self.kd = kd #derivative term
    
        self.limit_up = limitUp #upper saturation limit
        self.limit_down = limitDown #Lower saturation limit
        self.tau = tau #Time constant \tau of the differentiator

        self.int = 0 #Integrator state
        self.diff = 0 #Differentiator state 
        
        self.err_d1 = 0 #Previous error (_d1 means delayed by one time step)
        self.u = 0 #Last output 
        
        self.u_unsat = 0 #Last output unsaturated
        self.Ts = 0


    #implement integrator anti − windup
    def antiwindup(self):
        if (self.ki != 0):
            self.int = self.int +self.Ts/self.ki *(self.u - self.u_unsat)
   
    def initialize(self): #initialize variables
        self.int = 0
        self.diff = 0
        self.err_d1 = 0
       
    def run(self,  err, Ts):
        self.int = self.int + Ts/2*(err + self.err_d1)
        self.diff = ((2*self.tau-Ts)/(Ts+2*self.tau)*self.diff + 2/(2*self.tau +Ts)*(err-self.err_d1) )
        self.err_d1 = err
        
        self.u = self.kp*err + self.ki*self.int + self.kd*self.diff
        self.u_unsat = self.u
        self.Ts = Ts

        return self.u 
    
    def saturate(self):
         if (self.u > self.limit_up):
             self.u = self.limit_up
         elif (self.u < self.limit_down):
             self.u = self.limit_down
         
         return self.u 
    

      
