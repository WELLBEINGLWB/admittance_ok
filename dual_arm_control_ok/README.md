# Dual_arm_ dynamic control

## Admittance controller of a dual-arm panda system within Cooperative Dual Task Space.

This repository contains matlab code for the simulations in CoppeliaSim via RemoteApi functions of three different dynamic admittance controllers using Dual Quaternions of a dual arm system composed of 2 7dof Panda robots.

The controllers are designed within the Cooperative Dual Task Space in terms of absolute and relative pose variables.

A brief description of the three approaches is described below:

1. *full_dual_position_control.m* :
    A fixed reference for the relative pose between the two arms is given, while the absolute pose trajectory is computed by the external impedance loop
    to guarantee a compliant behaviour. 
    All 12 dofs are controlled by defining the augmented error as e = [er;ea], that is relative and absolute pose error respectively. 
    The inner motion control is a task-space inverse dynamics controller with full feedback linearization.
    The additional 2dofs are exploited to guarantee joints-limits avoidance. 
    
2. *decoupled_control.m*:
    References poses for each arm are computed directly from cooperative dual task-space variables definition.
    The arms are controlled separately trough two task-space inverse dynamics controllers.
    
3. *multi_priority_control.m*: 
    A multi-priority control at acceleration level is proposed: relative pose is defined as primary task, and then the controller torques of the absolute pose are     projected in the null-space of the relative pose Jacobian. 

The three controllers are tested on 2 simple demos of a free motion and interaction task.

The results obtained in terms of tracking performance of relative and absolute positions as well as external forces are compared in plots in the script
*perfomance_analysis.m*.

Instructions for running the code are inside the *read_me.txt* file in the folder.




