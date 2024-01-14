Gabrielle Wink
# ME 449: Mobile Manipulation Capstone Project

[LINK](https://hades.mech.northwestern.edu/index.php/ME_449_Robotic_Manipulation) to course website. 

## Assignment Description
This software plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile
base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves,
and performs feedback control to drive the youBot to pick up a block at a specified location,
carry it to a desired location, and put it down (3).


### Results Discussion:
The youBot was able to achieve the task specified in the assignment description. One unexpected behavior encounter was sometime the block sometimes exhibited small vibrations after being grasped by the robot. This is an artifact of dynamic simulation. The results for each simulation are further described and located in the results directory.

### Steps to Run Code:

#### General Steps:
1. Download and save best.py
2. Navigate to directory where the script is saved
3. Run: best.py
4. Follow the runscript instructions
5. Then, after running the code, the csv file will be saved in the same location as the script
6. Open CoppeliaSim Scene 6 and play the csv file produced by the code


#### Specific Usage Instructions:
The required code inputs, code outputs, and solution steps are clearly labeled in the code
files: best.py, newTask.py, and overshoot.py.

### Code Directory:
This directory contains best.py, newTask.py, and overshoot.py. These files output a CoppeliaSim .csv file of successive youbot configurations, an Xerr csv file, and a plot of the six elements of Xerr as a function of time. These files are essentially the same, however, have different inputs. These differences are specified in the README files located in the results directory.
In the code directory there is also a Milestones directory. This directory contains the three Milestones. These Milestones pass all the tests specified on the Project Wiki.

### Simulation:
CoppeliaSim Scene 6
physics engine: ODE

### Video Demonstration Comment:
The screencast software slowed down the simulation, therefore, the video demonstrations are slower that the actual total time of the motion of the youBot. When the simulation is running without using the screencast application the total time of motion of the youBot is 33 seconds, as expected from the time specifications in my TrajectoryGenerator Function.

### References
1. http://hades.mech.northwestern.edu/index.php/Writing_a_CSV_File
2. https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robo
tics/core.py
1. http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone 4. http://hades.mech.northwestern.edu/index.php/Modern_Robotics