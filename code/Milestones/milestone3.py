#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  1 18:14:57 2020

@author: gabbiewink
"""


import numpy as np
import modern_robotics as mr

def FeedbackControl(X, Xd, Xd_next, kp, ki, dt):
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
    
    """
    # going to get this from milestone 1 fucntion
    config_vec = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0]) ################################
    
    
    
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
    print("Vd: ", Vd)
    term1 = np.dot(AdT,Vd) # Xsb
    print("advd: ", term1)
    
    
    # Term 2 - configuration error 
    Xerr_hat = mr.MatrixLog6(Xinv_Xd)
    Xerr = mr.se3ToVec(Xerr_hat) # unhat
    print("Xerr: ", Xerr)
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
    V = term1 + term2 + term3
    print("V: " , V) # commanded end-effector twist
    

    # find the jacobian 
    # base jacobian
    J_base =  np.dot(mr.Adjoint(Teb),F6) # wheel contribution
    # arm jacobian (body jacobian)
    J_arm = mr.JacobianBody(Blist,thetalist) # arm joint contribution (aka body Jacobian)
    Je = np.concatenate((J_base, J_arm),axis =1) # [Jbase Jarm] , end-effector
    print("Je: ", Je)
    Je_sinv = np.linalg.pinv(Je, 1e-4) # pseudoinvese
    
 
    
    # [u, thetadot] , [wheel speeds u (4 variables) and arm joint speeds thetadot (5 variables) ] rad/s
    control_vec = np.dot(Je_sinv, V)  
    return control_vec
    

    
    



# test inputs
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




F = FeedbackControl(X, Xd, Xd_next, kp, ki, dt)
print("speeds: ", F)
# mr code
#AdT = mr.Adjoint(T)
#se3mat = mr.MatrixLog6(T)
#V = se3ToVec(se3mat)
# JacobianBody(Blist,thetalist)