#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  2 17:15:40 2020

@author: gabbiewink
"""
#1. generate a reference trajectory using TrajectoryGenerator
#2. set the initial robot config  [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state]
#3. loop through the reference trajectroy (Xd and Xd,next) N-1 times
#4. 
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt

def format_list(traj, grip):
    """Generates the reference trajectory for the end effector frame {e} in correct order
    :param Tse_i: The initial configuration of the end-effector in the reference trajectory
    :param Tsc_i: The cube's initial configuration
    :param Tsc_f: The cube's desired final configuration
    :param Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube
    :param Tce_standoff: The end-effector's standoff configuration above the cube
    :param k: The number of trajectory reference configurations per 0.01 seconds
    
    :return: NX13 list of lists each of these N reference points represents a 
    transformation matrix Tse of the end-effector frame {e} relative to {s} at 
    an instant in time, plus the gripper state (0 or 1)
    [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]
    """
    correct_step_list  = []
    for p in traj:
        z = p.tolist() # change vector from array to list
        # rotation matix values for each row
        r1 = z[0][0:3]
        r2 = z[1][0:3]
        r3 = z[2][0:3]
        # translation values for each row
        px = [z[0][3]]
        py = [z[1][3]]
        pz = [z[2][3]]
        # list is proper form for coppelia sim
        new_list = r1 + r2 +r3 + px + py + pz + [grip]
        correct_step_list.append(new_list)
        
    return correct_step_list # list of list of each end effector config for traj
        
            
    
def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k = 1):
    """Generates the reference trajectory for the end effector frame {e} relative to {s} frame
    :param Tse_i: The initial configuration of the end-effector in the reference trajectory
    :param Tsc_i: The cube's initial configuration
    :param Tsc_f: The cube's desired final configuration
    :param Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube
    :param Tce_standoff: The end-effector's standoff configuration above the cube
    :param k: The number of trajectory reference configurations per 0.01 seconds
    
    :return: NX13 Matrix each of these N reference points represents a 
    transformation matrix Tse of the end-effector frame {e} relative to {s} at 
    an instant in time, plus the gripper state (0 or 1)
    [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]
    
    
    Example Input:
        # initial end effector config in s frame...found from scene 8 in coppelia sim
        Tse_i =  np.array([[1, 0, 0,     0],
                           [0, 1, 0,     0],
                           [0, 0, 1,   0.5],
                           [0, 0, 0,     1]])
        
        # cube's initial config
        Tsc_i =  np.array([[1, 0, 0,     1],
                           [0, 1, 0,     0],
                           [0, 0, 1, 0.025],
                           [0, 0, 0,     1]])
        
        # cube's desired final config
        Tsc_f = np.array([[0,  1, 0,      0],
                          [-1, 0, 0,     -1],
                          [0,  0, 1,  0.025],
                          [0,  0, 0,      1]])
        
        # The end-effector's configuration relative to the CUBE when it is grasping the cube 
        Tce_grasp =  np.array([[ 0, 0, 1,     0], # origins of frame e aligned with c 
                               [ 0, 1, 0,     0],
                               [-1, 0, 0,     0],
                               [ 0, 0, 0,     1]])
        
        # end-effector's standoff configuration above the cube
        Tce_standoff =  np.array([[ 0, 0, 1,     0],
                                  [ 0, 1, 0,     0],
                                  [-1, 0, 0,   0.2], # HOVER above
                                  [ 0, 0, 0,     1]])
        
        # The number of trajectory reference configurations per 0.01 seconds
        k = 1 # 10 Hz
            
        # call function
        traj_lists = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k )
    Output:
        list of lists of the trajectory at each timestep

    """
    total_traj_lists = []
    method = 5
    grip_open = 0
    grip_close = 1
    
    # 1. A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
    Tse_standoff_i = np.dot(Tsc_i, Tce_standoff) # final position of end effector for traj 1 represented in s frame
    #compute traj
    traj1 = mr.ScrewTrajectory(Tse_i, Tse_standoff_i, 7, 700, method) # Xstart, Xend, Time, N, method
    return_1 = format_list(traj1, grip_open)
    total_traj_lists += return_1

    
    # 2. A trajectory to move the gripper down to the grasp position.
    Tse_grasp_i = np.dot(Tsc_i, Tce_grasp) # grasp position in relative to s frame
    traj2 = mr.CartesianTrajectory(Tse_standoff_i, Tse_grasp_i , 3, 300, method) #Xstart, Xend, Tf, N, method
    return_2 = format_list(traj2, grip_open)
    total_traj_lists += return_2
 
    # 3. Closing of the gripper.
    traj3 = mr.CartesianTrajectory( Tse_grasp_i, Tse_grasp_i , 2, 200, method) # maintain same position
    return_3 = format_list(traj3, grip_close)
    total_traj_lists += return_3
    
    # 4. A trajectory to move the gripper back up to the "standoff" configuration.
    traj4 = mr.CartesianTrajectory( Tse_grasp_i, Tse_standoff_i, 3, 300, method)
    return_4 = format_list(traj4, grip_close)
    total_traj_lists += return_4
    
    # 5. A trajectory to move the gripper to a "standoff" configuration above the final configuration.
    Tse_standoff_f = np.dot(Tsc_f, Tce_standoff)
    traj5 = mr.CartesianTrajectory( Tse_standoff_i, Tse_standoff_f, 10, 1000, method) 
    return_5 = format_list(traj5, grip_close)
    total_traj_lists += return_5

    # 6. A trajectory to move the gripper to the final configuration of the object.
    Tse_grasp_f = np.dot(Tsc_f, Tce_grasp) # grasp position in relative to s frame
    traj6 = mr.CartesianTrajectory(Tse_standoff_f, Tse_grasp_f , 3, 300, method) #Xstart, Xend, Tf, N, method
    return_6 = format_list(traj6, grip_close)
    total_traj_lists += return_6
    
    # 7. Opening of the gripper.
    traj7 = mr.CartesianTrajectory( Tse_grasp_f, Tse_grasp_f , 2, 200, method) # maintain same position
    return_7 = format_list(traj7, grip_open)
    total_traj_lists += return_7
    
    # 8. A trajectory to move the gripper back to the "standoff" configuration.
    traj8 = mr.CartesianTrajectory(Tse_grasp_f, Tse_standoff_f, 3, 300, method) #Xstart, Xend, Tf, N, method
    return_8 = format_list(traj8, grip_open)
    total_traj_lists += return_8
    
    return total_traj_lists

def NextState(config_vec, control_vec, dt, max_speed):
    """Simulator for the kinamatics of the robot
    1. Replaces any speed in the 9-vector of controls that is outside
      [-max_speed, max_speed] with nearest boundary of the range
    2. updates arm joint angles and chassis wheel angles
    3. computes new chasis configuration
    
    :param config_vec: 12-vector representing the current configuration of the robot
                       [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
    :param control_vec: 9-vector of controls indicating the arm joint and wheel speeds 
                        [thetadot (5 variables) and the wheel speeds u (4 variables)]
    :param dt: timestep
    :param max_speed: maximum angular speed of the arm joints and the wheels (rad/s)
 
    
    :return:  A 12-vector representing the configuration of the robot time Δt later
            [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
            
    
    Example Input:
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
       
        # Call Function
        new_config_vec = NextState(config_vec, control_vec, dt, max_speed)    
    
    Output:
        np.array([0.01233766, 0., 0., 0.01, 0., 0.21, -1.6, 0., -0.1, 0.1, 0.1, -0.1])
        
    
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



def FeedbackControl(config, Xd, Xd_next, kp, ki, dt):
    """Calculates the kinematic task-space feedforward plus feedback control law
    Frames (p.550 Modern Robotics)
    1. fixed space frame {s}
    2. chassis frame {b}
    3. frame at the base of the arm {o}
    4. end-effector frame {e}
    
    
    :param X: The current (actual) configuration of the end-effector (Tse(q,theta))
    :param Xd:  current end-effector reference configuration 
    :param Xd_next: end-effector reference configuration at the next timestep 
                    in the reference trajectory, Xd,next (i.e., Tse,d,next), at 
                    a time Δt later.
    :param Kp: gain matrix
    :param Ki: gain matrix
    :param dt: timestep between reference trajectory configurations
        
    :return: The commanded end-effector twist V expressed in the 
             end-effector frame {e}
             [u, thetadot]
    
             
    Input:
        config_vec = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
        dt = 0.01
        X = np.array([[ 0.170, 0, 0.985,   0.387], 
                      [     0, 1,     0,       0],
                      [-0.985, 0, 0.170,   0.570],
                      [ 0    , 0,     0,       1]])
        Xd = np.array([[ 0, 0, 1,     0.5], 
                       [ 0, 1, 0,       0],
                       [-1, 0, 0,     0.5],
                       [ 0, 0, 0,       1]])
        Xd_next = np.array([[ 0, 0, 1,     0.6], 
                            [ 0, 1, 0,       0],
                            [-1, 0, 0,     0.3],
                            [ 0, 0, 0,       1]])
        kp = 1
        kp_matrix = kp * np.identity(6)
        ki = 0
        ki_matrix = ki * np.identity(6)
    Output:
        np.array([ 1.57451207e+02  1.57451207e+02  1.57451207e+02  1.57451207e+02
                  4.23493788e-14 -6.54282780e+02  1.40086981e+03 -7.46757827e+02
                  2.07905230e-14])
    """
    # going to get this from milestone 1 fucntion
    # phi, x, y, theta1, theta2, theta3, theta4, theta5
    config_vec = config
    
    # GIVENS 
    l = 0.235  # 1/2 forward-backwards distance between wheels
    w = 0.15   # 1/2 side to side distance between wheels
    r = 0.0475 # wheel radius
    F6 =  (r/4) * np.array([[      0,       0,       0,        0],    # F6 = [0, 0, F, 0].T , mobile base Vb = Fu         
                           [       0,       0,       0,        0], 
                           [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],  
                           [       1,       1,       1,        1],
                           [      -1,       1,      -1,        1],
                           [       0,       0,       0,        0]])
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],   # screw axis of arm in end-effector frame
                      [0, -1, 0,  -0.5076,     0, 0],
                      [0, -1, 0,  -0.3526,     0, 0],
                      [0, -1, 0,  -0.2176,     0, 0],
                      [0,  0, 1,        0,     0, 0]]).T
    thetalist = config_vec[3:8] # arm joint angles
   
    # Transformations needed
    Tbo = np.array([[1, 0, 0, 0.1662],  # fixed offset of {o} from {b}
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])
    
    Moe= np.array([[1, 0, 0,  0.033],
                   [0, 1, 0,      0],
                   [0, 0, 1, 0.6546],
                   [0, 0, 0,      1]])
    Toe = mr.FKinBody(Moe, Blist, thetalist) # forward kinamatics of the arm 
    Teo = mr.TransInv(Toe)
    Tob = mr.TransInv(Tbo)
    Teb = np.dot(Teo,Tob)
    

    # find X = Tse = Tsb(q)TboToe(theta_list)
    # theta_list is arm joint positions
    Tsb = np.array([[np.cos(config_vec[0]), -np.sin(config_vec[0]),     0, config_vec[1]], 
                    [np.sin(config_vec[0]),  np.cos(config_vec[0]),     0, config_vec[2]],
                    [                    0,                      0,     1,        0.0963],
                    [                    0,                      0,     0,             1]])
    Tso = np.dot(Tsb, Tbo)
    Tse = np.dot(Tso, Toe)
    X = Tse
  

    # invert and multiply matrices
    X_inv = mr.TransInv(X)
    Xinv_Xd = np.dot(X_inv, Xd)
    Xd_inv = mr.TransInv(Xd)
    Xdinv_Xdnext = np.dot(Xd_inv, Xd_next)
    
    
    # TERM 1- Feedforward twist Vd in the actual end-effector frame rather than the desired end-effector frame
    AdT = mr.Adjoint(Xinv_Xd) 
    log_Xdinv_Xdnext = mr.MatrixLog6(Xdinv_Xdnext)
    Vd_hat =  (1/dt)* log_Xdinv_Xdnext  # divide by dt because [vd] = xd_inv*xd_dot
    Vd = mr.se3ToVec(Vd_hat)  # feedforward reference twist
    term1 = np.dot(AdT,Vd) # Xsb

    
    # Term 2 - configuration error 
    Xerr_hat = mr.MatrixLog6(Xinv_Xd)
    Xerr = mr.se3ToVec(Xerr_hat) # unhat
    Xerr_plot.append(Xerr)
    term2 =  np.dot(kp, Xerr)
    
    # Term 3
    total1 = 0
    total2 = 0
    total3 = 0
    total4 = 0
    total5 = 0
    total6 = 0
    total = [total1, total2, total3, total4, total5, total6]
    # adds Xerr*dt to a running total (p.429 modern robotics)
    err_int = Xerr*dt  # increment
    for ele in range(0, 6):
        err_int_list = err_int.tolist()
        total[ele] += total[ele] + err_int_list[ele]
        
    err_int = np.array(total)
    term3 = np.dot(ki, err_int)
 
    # kinematic task-space feedforward plus feedback control law
    V = term1 + term2 + term3 # commanded end-effector twist
    

    # find the jacobian (Modern Robotics ch.13.5)
    # base jacobian
    J_base =  np.dot(mr.Adjoint(Teb),F6) # wheel contribution
    # arm jacobian (body jacobian)
    J_arm = mr.JacobianBody(Blist,thetalist) # arm joint contribution (aka body Jacobian)
    Je = np.concatenate((J_base, J_arm),axis =1) # [Jbase Jarm] , end-effector
    Je_sinv = np.linalg.pinv(Je, 1e-4) # pseudoinvese
    
 
    
    # [u, thetadot] , [wheel speeds u (4 variables) and arm joint speeds thetadot (5 variables) ] rad/s
    control_vec = np.dot(Je_sinv, V)  
    return control_vec


# list of lists of Xerr for each timestep in order
Xerr_plot = []  

##############################################################################
# STEP 1- Generate a reference trajectory (milestone2)
#### INPUTS ### (desired traj setup)
# initial end effector config of refernce trajectroy
Tse_i =  np.array([[1, 0, 0,     0],
                   [0, 1, 0,     0],
                   [0, 0, 1,   0.5],
                   [0, 0, 0,     1]])

# cube's initial config
Tsc_i =  np.array([[1, 0, 0,     0.5],
                   [0, 1, 0,     0],
                   [0, 0, 1, 0.025],
                   [0, 0, 0,     1]])

# cube's desired final config
Tsc_f = np.array([[1,  0, 0,      2.5],
                  [0,  1, 0,      0],
                  [0,  0, 1,  0.025],
                  [0,  0, 0,      1]])

# The end-effector's configuration relative to the CUBE when it is grasping the cube 
Tce_grasp =  np.array([[ 0, 0, 1,     0], # origins of frame e aligned with c 
                       [ 0, 1, 0,     0],
                       [-1, 0, 0,     0],
                       [ 0, 0, 0,     1]])

# end-effector's standoff configuration above the cube
Tce_standoff =  np.array([[ 0, 0, 1,     0],
                          [ 0, 1, 0,     0],
                          [-1, 0, 0,   0.2], # HOVER above
                          [ 0, 0, 0,     1]])

# The number of trajectory reference configurations per 0.01 seconds
k = 1 # 10 Hz
  
### call function ####
# returns list of list: [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]
traj_lists = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k )

##############################################################################
# Step 2- set initial robot configuration
####Inputs###### (actual config setup)

#the current configuration of the robot - chassis config and joint angles of arm
chassis_phi = np.pi/3 # orientation error
chassis_x = .2 # position error
chassis_y = 0
J1 = 0
J2 = 0
J3 = 0 # rads
J4 = 0
J5 = 0
W1 = 0
W2 = 0
W3 = 0
W4 = 0
# timestep between reference trajectroy configurations
dt = 0.01 
# max angular speed of arm joints and wheels
max_speed = 12.3  # rad/s ()

### call function ###
initial_config_vec = np.array([chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4])


###############################################################################
# Step 3 - design the controller
### Inputs ###
#kp = ki = 0 for feedforward control

kp = 20 # proportional gain
ki = 15 # intergral gain 


## do not modify - gain matices computed from kp and ki inputs################
kp_matrix = kp * np.identity(6)
ki_matrix = ki * np.identity(6)
##############################################################################
# MAIN LOOP to find configs
# loops through reference trajectory
# N reference configurations
# the loop runs N-1 , Xd = N, Xd,next = N+1
# ex. if N = 10 the tenth time through the loop nthe controller use the 10th config as Xd and the 11th as Xdnext to find Vd

# to execute program
value = input("To run the script please enter any string: \n")
print('You entered: ', value)
if type(value) == str:
    print("runscript")
    N = len(traj_lists)
    all_config_lists = []
    for i in range(N-1):
        # generate wheel and joint controls 
        Xd = np.array([[traj_lists[i][0], traj_lists[i][1], traj_lists[i][2], traj_lists[i][9]], 
                       [traj_lists[i][3], traj_lists[i][4], traj_lists[i][5], traj_lists[i][10]],
                       [traj_lists[i][6], traj_lists[i][7], traj_lists[i][8], traj_lists[i][11]],
                       [               0,                0,                0,                 1]])
        Xd_next = np.array([[traj_lists[i+1][0], traj_lists[i+1][1], traj_lists[i+1][2], traj_lists[i+1][9]], 
                            [traj_lists[i+1][3], traj_lists[i+1][4], traj_lists[i+1][5], traj_lists[i+1][10]],
                            [traj_lists[i+1][6], traj_lists[i+1][7], traj_lists[i+1][8], traj_lists[i+1][11]],
                            [                 0,                  0,                  0,                   1]])
            
        # initial config
        if i == 0:
            initial_config = initial_config_vec
            config_vec = initial_config
        control_vec = FeedbackControl(config_vec[0:8], Xd, Xd_next, kp, ki, dt)
        # control variables
        thetadot_list = control_vec[4:9].tolist() # theta1_dot, theta2_dot, ..., theta5_dot 
        u_list = control_vec[0:4].tolist() # wheel driving angular speed: wheel1 to wheel 4, respectively
        control_list = thetadot_list + u_list
        control_vec = np.array(control_list)
        
        # add configuration used for iteration to config_vec_lists
        config_list = config_vec.tolist()
        config_list.append(traj_lists[i][12]) # add grip state to config vector
        all_config_lists.append(config_list)  # add all_config_list 
        
        
        # calculate new configuration for next loop 
        new_config = NextState(config_vec, control_vec, dt, max_speed) #np.array([chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4])
        config_vec = new_config
    
    ###############################################################################
    print("Generating animation csv file.")
    # csv file of configurations
    all_config_matrix = np.array(all_config_lists)
    tm = all_config_matrix
    # writing a csv
    # Open a file for output
    # Overwrite
    f = open("newTask_congurations.csv", "w") 
    # For loop running print each csv row
    for i in range(len(all_config_lists)):
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
    
    ##########################################################################
    # PLOT Xerr
    
    t0 = 0
    tf = len(all_config_lists) *dt
    
    plott = np.arange((tf-t0)/dt)*dt
    plt.plot(plott, Xerr_plot) # trajectories of config vars
    plt.xlabel('time (s)')
    plt.ylabel('Xerr 6-Vector')
    plt.legend(['werr_x(1)','werr_y(2)','werr_z(3)','verr_x','verr_y','verr_z'])
    plt.grid(True)
    plt.show()
    
    
    ##########################################################################
    print("Writing error plot data.")
    # create csv data file of Xerror
    all_error_matrix = np.array(Xerr_plot)
    tm = all_error_matrix
    # writing a csv
    # Open a file for output
    # Overwrite
    f = open("Xerr.csv", "w") 
    # For loop times to print each csv row
    for i in range(len(Xerr_plot)):
        #output = " %10.6f, %10.6f, %10.6f, %10.6f, %d\n" % (j[i,0], j[i,1], j[i,2], j[i,3], d[i])   #four joints
        # add more joints in list if needed
        output = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n " %(tm[i,0], 
                                                                        tm[i,1], 
                                                                        tm[i,2],
                                                                        tm[i,3], 
                                                                        tm[i,4],
                                                                        tm[i,5])
        f.write(output)
            
    # close file
    f.close()
    ##########################################################################
    print("Done.")