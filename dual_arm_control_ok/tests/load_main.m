%% Run for simulink tests
%% Main script

close all;
clear all;
clear classes;
warning off;
clc;


%% Build robot
FEp_DH_theta  = [0,     0,      0,      0,        0,        0,      0];
FEp_DH_d      = [0.333, 0,      0.316,  0,        0.384,    0,      0.107];
FEp_DH_a      = [0,     0,      0.0825, -0.0825,  0,        0.088   0.0003];
FEp_DH_alpha  = [-pi/2, pi/2,   pi/2,   -pi/2,    pi/2,     pi/2    0];
franka_dh_matrix = [FEp_DH_theta;FEp_DH_d; FEp_DH_a;FEp_DH_alpha];
franka1 = DQ_SerialManipulator(franka_dh_matrix,'standard');
franka2 = DQ_SerialManipulator(franka_dh_matrix,'standard');


%%base of robots
base1 = normalize(DQ([0.707;0;0;0.7071;0.0035;-0.2506;-0.2506;-0.0035]));
base2 = normalize(DQ([0.707;0;0;-0.7071;-0.0035;-0.2506;0.2506;-0.0035]));
b1 = base1.q;
b2 = base2.q; 
franka1.set_reference_frame(base1);
franka2.set_reference_frame(base2); 

%% Joint limits
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Constants
C8 = diag([1, -1, -1, -1, 1, -1, -1, -1]);
threshold = 1e-12; 
Ts = 0.01;

%% Initial conditions
q_in_1 = [0.4670   -0.1818    0.1521   -2.2022   -2.1767    2.4575    2.3293];
q_in_2 = [-0.4654   -0.1791   -0.1528   -2.1994    2.1769    2.4577   -2.3291]; 
q_in = [q_in_1;q_in_2]; 

%dual-arm
two_arms =  DQ_CooperativeDualTaskSpace(franka1, franka2);
x_rel_in = vec8(relative_pose(two_arms,q_in)); %relative pose
xa_in = vec8(absolute_pose(two_arms,q_in)); %absolute pose

%desired absolute pose
xa_d = vec8(DQ(xa_in).P + 0.5*DQ.E*(DQ([0;0.0543;0.0139;0.4872])*DQ(xa_in).P));

%% Impedance controller
xr_in = xa_in; 
xd_in = xa_in; 
e_in = vec8(DQ(xr_in)'*DQ(xd_in));
yr_in = vec6(log(DQ(e_in)));
dyr_in = [0 0 0 0 0 0]';

%% motion controller 
err_in = zeros(8,1); 

%% Variable gain utils
berta = 0.98;
inc = 1000;
csi = 0.4;
a0 = 0.99;
k_def = 100; %N/m arbitrarly low gain for interaction 
mass = 1; %kg
e_max = 0.01;
F_max = [1;1]; % N
ov_max = 0.1;
tsettl_max = 0.5; %sec
ts_bound = 0.05;
alpha = 0.5;
F_int_max = [5;5]; % N
disp('Loading complete') 

