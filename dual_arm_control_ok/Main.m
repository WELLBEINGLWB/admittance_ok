%% Main script file
%% Clear workspace
clear;
close all;
clc;

disp('Loading parameters..')

%% Addpath
p1 = genpath('Vrep_utils');
p2 = genpath('functions');
p3 = genpath('other'); 
p4 = genpath('data');
p5 = genpath('tests');
addpath(p1,p2,p3,p4,p5); 

%% Define task
%fuse = 1 --> free motion
%fuse = 2 --> interaction task
fuse = 1; 

%% Parameters
cdt = 0.01; %[s] sampling time
tfin = 2.5; %s 
time = 0:cdt:tfin; %simulation time
tt = time; 

%% Panda Joints limits 
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];
qc = [0.5*(q_min+q_max)']; 

%% Impedance matrices
I = eye(6);
if fuse == 1
    Md1 = 1.5*I; %desired mass
    Kd1 = 1000*I; %desired stiffness
    Bd1 = sqrt(4*Kd1*Md1); %desired damping
else
    Md1 = 1.5*I; %desired mass
    Kd1 = 300*I; %desired stiffness
    Bd1 = 4*sqrt(4*Kd1*Md1); %desired damping
end

%% Motion controller gains
kp = 300;
kd = 30;
ki = 100;

%% Variable gain utils
%parameters
k_in = 300; %[N/m]
d_in = sqrt(4*300); %[Ns/m]
k_min = 30; %[N/m]
mass = 1; %kg desired mass
k_default = 1000; %N/m
d_default = sqrt(4*k_default); 

%%upper bound kdot
%% Variable gain utils
berta = 0.98;
inc = 1.08;
csi = 1;
a0 = 0.99;

%% Define controller type
contr = 1; %--> full dual_position
% contr = 2; %--> multi_priority control

disp('Done!')