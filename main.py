# MAE 6245 - Robotics System | (Spring 2021)
#
# Class Project - Quadrotor dynamics, control and simulation (plus configuation)
#
# Group members: Maneesha Wickramasuriya (G46823188)(maneesh@gwu.edu)
#                Shruty Balaji (G32181536)(shruty808@gwu.edu)
#
# ###  RUN THIS FILE
# main.py (Version 1.0) 
#___________________________________________________________________________


import Quad
import matplotlib.pyplot as plt
import matplotlib.animation as animation
while(1):
    flag = Quad.update(1)
    if flag is not True:
       break

    #ani = animation.FuncAnimation(Quad.p.fig, Quad.update,interval=1)
    #plt.show()
