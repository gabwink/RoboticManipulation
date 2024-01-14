#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 29 17:32:35 2020

@author: gabbiewink
"""


import numpy as np
import modern_robotics as mr

def NextState(config_vec, control_vec, dt, max_speed):
    """Simulator for the kinamatics of the robot
    1. Replaces any speed in the 9-vector of controls that is outside
      [-max_speed, max_speed] with nearest boundary of the range
    2. updates arm joint angles and chassis wheel angles
    3. computes new chasis configuration
    
    :param config_vec: 12-vector representing the current configuration of the robot
                       [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
    :param control_vec: 9-vector of controls indicating the arm joint speeds 
                        [thetadot (5 variables) and the wheel speeds u (4 variables)]
    :param dt: timestep
    :param max_speed: maximum angular speed of the arm joints and the wheels (rad/s)
 
    
    :return:  A 12-vector representing the configuration of the robot time Δt later
            [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
    """
    # catches any speed out on bound [max_speed, -max_speed] and sets it to nearst boundary
    count = 0
    for speed in control_vec:
        if speed > max_speed:
            control_vec[count] = max_speed
        elif speed < -max_speed:
            control_vec[count] = -max_speed
        count +=1
    
    #first-order Euler step: new arm and wheel config
    new_jw_list = [] # j: arm config, w: wheel angles
    thetadot_list = []
    i = 0
    for config in config_vec[3:12]:
        #new arm joint angles = (old arm joint angles) + (joint speeds) * Δt
        #new wheel angles = (old wheel angles) + (wheel speeds) * Δt
        new_config = config + control_vec[i] * dt
        new_jw_list.append(new_config)
        
        # once get to wheel part of config_vec: find wheel displacements and thetadot
        if i > 4:
            thetadot = (new_config - config)/ dt 
            thetadot_list.append(thetadot)
        i += 1
            
    #new chassis configuration is obtained from odometry, as described in Chapter 13.4
    # 1. measure wheel displacement
    # 2. assume constant wheel speeds so thetadot = deltatheta/dt
    # 3. find twist in b frame (chasis) Vb = F*thetadot
    # 4. T_bkbk+1 = e[Vb6]
    # 5. T_sbk+1 = Tsbke[vb6]
    # CHASIS VARS - given on p.4 of wiki project instructions
    l = 0.235 # 1/2 forward-backwards distance between wheels
    w = 0.15 # 1/2 side to side distance between wheels
    r = 0.0475 # wheel radius
    x = config_vec[1]
    y = config_vec[2]
    phi = config_vec[0]
    T_sb =  np.array([[np.cos(phi), -np.sin(phi), 0,      x], 
                      [np.sin(phi),  np.sin(phi), 0,      y],
                      [          0,            0, 1, 0.0963],
                      [          0,            0, 0,      1]])
    
    # pseudo inverse of H(0) for four-mecanum-wheel robot (Modern Robotics p. 549)
    F =  (r/4) * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],  
                           [       1,       1,       1,        1],
                           [      -1,       1,      -1,        1]])
    
    deltatheta = np.array(thetadot_list) * dt
    Vb = np.dot(F, deltatheta) #body twist vector (wbz, vbx, vby)
    
    
    Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0]) #[0, 0, wbz, vbx, vby, 0]
    se3mat = mr.VecTose3(Vb6)
    T_bbf= mr.MatrixExp6(se3mat)
    T_sbf = np.dot(T_sb, T_bbf)
    chassis_phi_new = np.arccos(T_sbf[0,0])
    chassis_x_new = T_sbf[0,3]
    chassis_y_new = T_sbf[1,3]

    # new config_vec
    return np.array([chassis_phi_new, chassis_x_new, chassis_y_new, 
                     new_jw_list[0], new_jw_list[1], new_jw_list[2], new_jw_list[3], new_jw_list[4], 
                     new_jw_list[5], new_jw_list[6], new_jw_list[7], new_jw_list[8]])

    
# configuration vairables
chassis_phi = 0
chassis_x = 0
chassis_y = 0
J1 = 0
J2 = 0
J3 = 0.2 # rads
J4 = -1.6
J5 = 0
W1 = 0
W2 = 0
W3 = 0
W4 = 0

# control variables
thetadot_list = [1, 0, 1, 0, 0] # theta1_dot, theta2_dot, ..., theta5_dot 
#u_list = [10, 10, 10, 10] # wheel driving angular speed: wheel1 to wheel 4, respectively
u_list = [-10, 10, 10, -10]
control_list = thetadot_list + u_list

# FUNCTION INPUTS
dt = 0.01 
max_speed = 15  # rad/s
config_vec = np.array([chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4])
control_vec = np.array(control_list)

new_config_vec = NextState(config_vec, control_vec, dt, max_speed)
print(new_config_vec)


# make list of np.array of configs from function
config_vec_list = []
for i in range(101):
    config_vec_list.append(config_vec) 
    new_config_vec = NextState(config_vec, control_vec, dt, max_speed) # config_vec changes each loop
    config_vec = new_config_vec
    
# add 0 for open grip to vectors in list and change type to list
config_grip_list = []
for vec in config_vec_list:
    vec_list = vec.tolist()
    vec_list.append(0)
    config_grip_list.append(vec_list)

config_matrix = np.array(config_grip_list)
tm = config_matrix
# writing a csv
# Open a file for output
# Overwrite
f = open("milestone1.csv", "w") 
# For loop running 6 times to print each csv row
for i in range(len(config_grip_list)):
    #output = " %10.6f, %10.6f, %10.6f, %10.6f, %d\n" % (j[i,0], j[i,1], j[i,2], j[i,3], d[i])   #four joints
    # add more joints in list if needed
    output = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f ,%10.6f\n " % (tm[i,0], 
                                                                                                                             tm[i,1], 
                                                                                                                             tm[i,2],
                                                                                                                             tm[i,3], 
                                                                                                                             tm[i,4],
                                                                                                                             tm[i,5],
                                                                                                                             tm[i,6], 
                                                                                                                             tm[i,7], 
                                                                                                                             tm[i,8],
                                                                                                                             tm[i,9], 
                                                                                                                            tm[i,10],
                                                                                                                            tm[i,11],
                                                                                                                            tm[i,12])
    f.write(output)
        
# close file
f.close()
   
  
   