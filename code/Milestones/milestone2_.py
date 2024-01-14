#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 17:01:31 2020
@author: gabbiewink


Steps to to run the code and generate a csv file.
1. Download and save milestone_2_edit.py
2. Navigate to directory where the script is saved
3. Run: python3 milestone.py
4. Then, after running the code, the csv file will be save in the same location 
   as the script 
"""

import numpy as np
import modern_robotics as mr


def format_list(traj, grip):
    """Generates the reference trajectory for the end effector frame {e}
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
    """Generates the reference trajectory for the end effector frame {e}
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
    traj5 = mr.CartesianTrajectory( Tse_standoff_i, Tse_standoff_f, 8, 800, method)
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
    
  
    
    traj_matrix = np.array(total_traj_lists)
    tm = traj_matrix
    
    # writing a csv
    # Open a file for output
    # Overwrite
    f = open("iterates_eight.csv", "w") 
    # For loop running 6 times to print each csv row
    for i in range(len(total_traj_lists)):
        
        
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
    
    
    return total_traj_lists



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

