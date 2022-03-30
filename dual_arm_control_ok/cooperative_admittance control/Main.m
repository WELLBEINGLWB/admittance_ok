%% Main script file
%% Clear workspace
clear;
close all;
clc;

disp('Loading parameters..')

%% Parameters
cdt = 0.01; %[s] sampling time
tfin = 2.5; %s 
time = 0:cdt:tfin; %simulation time
tt = time; 

%% Fixed vs adapting
fuse = 1; %nominal task
% fuse = 2; %additional weight

%% Panda Joints limits 
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];
qc = 0.5*(q_min+q_max)'; 

%% Impedance matrices
%absolute impedance
mass = 1.5;  %kg 
I = eye(6);
Md1 = 1.5*I; %desired mass
Kd1 = 300*I; %desired stiffness
Bd1 = sqrt(4*Kd1*Md1); %desired damping
%relative impedance
Md2 = 1.5*I; 
Kd2 = 100*I; %desired stiffness
Bd2 = sqrt(4*Kd2*Md2); %desired damping

%% Motion controller gains
kp = 300;
kd = 30;
ki = 100;

%% Grasped object
g = 9.81; %m/s^2
mass_obj = 0.5; %kg
weight = mass_obj*g; %N

%friction coefficient
mu_nom = 0.4;
alpha = 0.2; %uncertainty of 20%
mu_wc = mu_nom*(1-alpha); %worst case 

%% Grasping + lifting
%Parameters
g = 9.81; %m/s^2
mass_obj = 0.5; %kg
l_obj1 = 0.21; %m 
l_obj = 0.15; %m length (COM center of two arms(Ycm =0)
k_table = 5000; %N/m, environment stiffness
k_obj = 500; %N/m, object stiffness

disp('Done!')