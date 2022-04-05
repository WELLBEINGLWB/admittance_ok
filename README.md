# Admittance_ok

## Brief description
This repository contains an admittance controller for bimanual and cooperative manipulators (**CDAC**). 
The control is designed to be consistent with the geometry of the *Coperative task space* using the dual quaternion logarithmic mapping of wrenches and corresponding elastic displacements.  
The overall scheme consists of a stiffness adapter, cooperative admittance controller, and wrench adapter complete with an inner motion controller along with the cooperative system.

## Simulation
The CDAC is tested on a grasping task for two 7dof Panda robots by using CoppeliaSim via remApi functions in Matlab. 




