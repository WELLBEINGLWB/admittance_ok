## DUAL-ARM DYNAMIC CONTROL

Part 1:
Description: simulation of 2 tracking tasks (free-motion and interaction task) in CoppeliaSim of an admittance 
controller for a dual-arm system composed of 2 7-dof Panda Robots using Cooperative Dual Task Space frameworks.

The dual-arm system is controlled trough three different approaches and then the results in terms of tracking perfomance of the CDTS
variables are compared.

Instructions to run the simulations: 

1) Open the corresponding CoppeliaSim scene inside the folder:

   - dual_arm.ttt 

2) Set the value of flag fuse inside Main.m file to see the corresponding demo:

   fuse = 1 --> free motion tracking of a minimum jerk trajectory (y-z plane);
   fuse = 2 --> interaction task with an hard environment.
   
3) run Main.m

4) run the desired dynamic controller: 1) full_dual_position_control.m 
                                       2) decoupled_control.m
                                       3) multi_priority_control.m 

To visualize the plots of the results of the three different simulation run performance_analysis.m 

Obs: To change the simulation time and sampling time simply modify the values of time and cdt respectivately in main.m file.
     To try different trajectories with the same minimum jerk interpolation simply modify values of time and pos_i,pos_f inside the 
     for cicles in the correspondin .m files. 

## Part 2:
Description: simulation of one task the same admittance controller for the dual-arm system considering varying stiffness and damping gains.
The idea is to modulate the gains on each dof separately: lower stiffness and damping gains during interaction to guarantee safe interaction 
while a stiffer behaviour during free motion to achieve good tracking performances. 
