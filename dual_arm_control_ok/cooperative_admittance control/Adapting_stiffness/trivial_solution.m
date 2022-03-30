%% Cooperative dual-arm manipulators controller:
%% Admittance control on absolute pose of dual-arm system. 
%% Simulation: grasping task

include_namespace_dq;

%% Initialize variables
%%admittance variables absolute pose
xc1_data = zeros(size(time,2),8);
dxc1_data = zeros(size(time,2),8);
ddxc1_data = zeros(size(time,2),8);
yr1_data = zeros(size(time,2),6);
dyr1_data =  zeros(size(time,2),6);

%%wrench vectors
wa_ext_data = zeros(size(time,2),6); %external wrench mapped to abs task-space (world_frame)
wr_ext_data = zeros(size(time,2),6); %external wrench mapped to rel task-space  (world_frame)

psi_ext_a_data = zeros(size(time,2),6); %external wrench mapped to abs task-space (xa frame)
psi_ext_r_data = zeros(size(time,2),6); %external wrench mapped to abs task-space (xr frame)

%% Initialize V-REP interface

vi = DQ_VrepInterface;
vi.disconnect_all();
vi.connect('127.0.0.1',19997);
clientID = vi.clientID;
sim = vi.vrep;


%% Initialize VREP Robots
fep_vreprobot1 = FEpVrepRobot1('Franka1',vi);
fep_vreprobot2 = FEpVrepRobot2('Franka2',vi);
disp(' ');
disp('============== Robot Reference Frames (DQs) ==============')
disp(' ');

%% Load DQ Robotics kinematics
fep1  = fep_vreprobot1.kinematics();
fep2  = fep_vreprobot2.kinematics();

%% Build dual-arm system
panda_bimanual = DQ_CooperativeDualTaskSpace(fep1,fep2);

%% Get Joint Handles

handles = get_joint_handles(vi,vi.clientID);
joint_handles1 = handles.armJoints1;
joint_handles2 = handles.armJoints2;
disp(' ');
disp('============== Initial State of the Cooperative System ==============')
disp(' ');
% get initial state of the robot (Using VRep Matlab Remote API directly)
qstr = '[ ';
qdotstr = '[ ';

for j=1:7
    [res,q(j)] = vi.vrep.simxGetJointPosition(vi.clientID,joint_handles1(j),vi.vrep.simx_opmode_buffer);
    [res,qdot(j)] = vi.vrep.simxGetObjectFloatParameter(vi.clientID,joint_handles1(j),2012,vi.vrep.simx_opmode_buffer);
    qstr = [qstr,num2str(q(j)),' '];
    qdotstr = [qdotstr,num2str(qdot(j)),' '];
end

qstr = [qstr,']'];
qdotstr = [qdotstr,']'];
disp('Initial Joint positions for Franka1: ');
disp(qstr);
disp('Initial Joint velocities for Franka1: ');
disp(qdotstr);
qstr = '[ ';
qdotstr = '[ ';

for j=1:7
    [res,q(j)] = vi.vrep.simxGetJointPosition(vi.clientID,joint_handles2(j),vi.vrep.simx_opmode_buffer);
    [res,qdot(j)] = vi.vrep.simxGetObjectFloatParameter(vi.clientID,joint_handles2(j),2012,vi.vrep.simx_opmode_buffer);
    qstr = [qstr,num2str(q(j)),' '];
    qdotstr = [qdotstr,num2str(qdot(j)),' '];
end

qstr = [qstr,']'];
qdotstr = [qdotstr,']'];
disp(' ');
disp('Initial Joint positions for Franka2: ');
disp(qstr);
disp('Initial Joint velocities for Franka2: ');
disp(qdotstr);

%% Get initial state of the robots using DQ functions

q1_in = fep_vreprobot1.get_q_from_vrep();
q2_in = fep_vreprobot2.get_q_from_vrep();

%% Initial conditions
xin_1 = fep1.fkm(q1_in);
xin_2 = fep2.fkm(q2_in);

%%initial position
p_in_1 = xin_1.translation.q(2:4);
p_in_2 = xin_2.translation.q(2:4);

%%initial orientation
r0_1 = xin_1.rotation;
r0_2 = xin_2.rotation;

q_in = [q1_in;q2_in]; %joints vector

%%initial relative pose
xr_in = panda_bimanual.relative_pose(q_in);
xa_in = panda_bimanual.absolute_pose(q_in);

r0 = vec4(xa_in.P); %orientation of absolute frame
r0_r = vec4(xr_in.P); %orientation of relative frame

%% Desired absolute and relative pose trajectories
[xa_d,dxa_d,ddxa_d,xr_d,dxr_d,ddxr_d,grasp_data,phase_data] = gripper_traj(xin_1,xin_2,time);

%% Setting to synchronous mode
%---------------------------------------
sim.simxSynchronous(clientID,true);
sim.simxSynchronousTrigger(clientID);
sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, cdt, sim.simx_opmode_blocking);

%% Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
i = 1;

%% Get joints positions
%---------------------------------------
%Arm1
for j=1:7
    [~,qmread1(j)]  = sim.simxGetJointPosition(clientID,joint_handles1(j),sim.simx_opmode_blocking);
end

qm1 = double([qmread1])';

%Arm2
for j=1:7
    [~,qmread2(j)]  = sim.simxGetJointPosition(clientID,joint_handles2(j),sim.simx_opmode_blocking);
end

qm2 = double([qmread2])'; 

%stacked joint angles vector
qm = [qm1;qm2];

Ja = panda_bimanual.absolute_pose_jacobian(qm);
Jr = panda_bimanual.relative_pose_jacobian(qm); 

%% Prepare data
data.qm = [];  data.qm_dot = []; 
data.tau1_send = []; data.tau1_read = []; data.tau2_send = []; data.tau2_read = []; 
data.f_ext = []; data.fa_ext = []; data.fr_ext = [];  data.Ma = []; data.Mr = []; data.f1_ext = []; data.f2_ext = []; 
data.x1 = []; data.x2 = []; data.pos1 = []; data.pos2 = []; 
data.xr_des = [];  data.xr = [];  data.xa = []; data.xa_des = []; data.pos1_c = []; data.pos2_c = [];
data.pos_a = [];  data.pos_abs_d = [];
data.pos_r = []; data.pos_r_d = []; 
data.kd1 = []; data.bd1 = []; 
data.kd2 = []; data.bd2 = []; 
data.norm = [];

% time
inittime = sim.simxGetLastCmdTime(clientID) %retrieves the simulation time (ms) of the last fetched command

%% Control loop

while sim.simxGetConnectionId(clientID)~=-1

    if i>size(time,2)
        break
    end

    %% Getting current joint position and velocities

    %Panda1
    for j=1:7
        [~,qmread1(j)]  = sim.simxGetJointPosition(clientID,joint_handles1(j),sim.simx_opmode_blocking);
    end

    qmOld1 = qm1;
    Ja_old = Ja;
    Jr_old = Jr; 

    qm1 = double([qmread1])';
    qm_dot1 = (qm1-qmOld1)/cdt;

    %Panda2
    for j=1:7
        [~,qmread2(j)]  = sim.simxGetJointPosition(clientID,joint_handles2(j),sim.simx_opmode_blocking);
    end

    qmOld2 = qm2;
    qm2 = double([qmread2])';
    
    qm_dot2 = (qm2-qmOld2)/cdt;

    %Current dual-arm joint configuration
    qm = [qm1;qm2];
    qm_dot = [qm_dot1;qm_dot2];

    %Current EE poses
    x1 = vec8(panda_bimanual.pose1(qm));
    x2 = vec8(panda_bimanual.pose2(qm));

    %Current relative and absolute pose
    xr = vec8(panda_bimanual.relative_pose(qm));
    xa = vec8(panda_bimanual.absolute_pose(qm));

    %Get jacobians
    J1 =  panda_bimanual.pose_jacobian1(qm);
    J2 =  panda_bimanual.pose_jacobian2(qm);
    Jr = panda_bimanual.relative_pose_jacobian(qm); 
    Ja = panda_bimanual.absolute_pose_jacobian(qm);

    dxr = Jr*qm_dot; 
    dxa = Ja*qm_dot; 
    
    %Jacobian derivatives
    Jr_dot = (Jr-Jr_old)/cdt;
    Ja_dot = (Ja-Ja_old)/cdt; 
    
    %Store data
    data.qm(:,i) = qm;  data.qm_dot(:,i) = qm_dot;  data.xr(:,i) = xr; 
    data.x1(:,i) = x1; data.x2(:,i) = x2;  data.xa(:,i) = xa; 
    data.pos1(:,i) = vec4(DQ(x1).translation); 
    data.pos2(:,i) = vec4(DQ(x2).translation);
    data.pos_r(:,i) = vec4(DQ(xr).translation); 

    %Get dynamics
    M1 = get_MassMatrix(qm1);
    M2 = get_MassMatrix(qm2);
    g1 = get_GravityVector(qm1);
    g2 = get_GravityVector(qm2);
    c1 = get_CoriolisVector(qm1,qm_dot1);
    c2 = get_CoriolisVector(qm2,qm_dot2);
    M = blkdiag(M1,M2);
    g = [g1;g2];
    c = [c1;c2];
    
    %% Impedance control
    %% Admittance control
    % initialize variables
    if i~=1
        % abs pose
        xr1_adm = xc1_data(i-1,:)';
        yr1_in = yr1_data(i-1,:)';
        dyr1_in = dyr1_data(i-1,:)';
    else
        % abs pose
        xr1_adm = vec8(xa_in);
        e_in1 = vec8(DQ(xr1_adm)'*DQ(xa_d(1,:)));
        yr1_in = vec6(log(DQ(e_in1)));
        dyr1_in = zeros(6,1);
    end
    
    %External wrenches on arm1 and arm2 (world-frame)
    [wr_1,wr_2] = wrench_ext_grasp_task(x1,x2,grasp_data(i,:),time(i),fuse);
    
    %measured external forces
    f1 = wr_1(1:3,:);
    f2 = wr_2(1:3,:);
    
    %store values
    data.f1_ext(i,:) = f1;
    data.f2_ext(i,:) = f2; 
    
    %wrenches linear mapping to absolute and relative task-spaces

    [Wa,Wr] = wrench_mapping(wr_1,wr_2,x1,x2,xa);
    
    r0 = vec4(xa_in.P); 
    r0_1 = vec4(xr_in.P); 

    wa_ext_data(i,:) = Wa;
    wr_ext_data(i,:) = Wr; 
    
    %express wrenches wrt to absolute and relative frames 
    psi_ext_a = vec6(DQ(r0)'*DQ(Wa)*DQ(r0));
    psi_ext_a_data(i,:) = psi_ext_a; 
   
    %ext forces abs task space
    data.fa_ext(i,:) = wa_ext_data(i,1:3)';
    data.Ma(i,:) = wa_ext_data(i,4:6)';

    %ext forces rel task space
    data.fr_ext(i,:) = wr_ext_data(i,1:3)';
    data.Mr(i,:) = wr_ext_data(i,4:6)';

    %Desired absolute pose variables
    xa_des = xa_d(i,:)'; 
    dxa_des = dxa_d(i,:)';
    ddxa_des = ddxa_d(i,:)';

    %Desired relative pose variables
    xr_des = xr_d(i,:)'; 
    dxr_des = dxr_d(i,:)';
    ddxr_des = ddxr_d(i,:)';

    %%retrieve position
    pos_a = vec4(DQ(xa).translation); %current absolute position
    p_a = [pos_a(2);pos_a(3);pos_a(4)]; 
   
        
    %% Stiffness and damping
    %% Admittance control 
    %abs pose
    [xd,dxd,ddxd,yr,dyr] = admittance_control(xa_des,dxa_des,ddxa_des,psi_ext_a',xr1_adm,yr1_in,dyr1_in,Md1,Kd1,Bd1,time);
   
    
    %store values
    xc1_data(i,:) = xd; 
    dxc1_data(i,:) = dxd;
    ddxc1_data(i,:) = ddxd;
    yr1_data(i,:) = yr; 
    dyr1_data(i,:) = dyr; 
    

    xd_des = xc1_data(i,:)';
    dxd_des = dxc1_data(i,:)';
    ddxd_des = ddxc1_data(i,:)'; 

    xd2_des = xr_des;
    dxd2_des = dxr_des;
    ddxd2_des = ddxr_des; 
    
    %save data
    %absolute pose 
    data.xa_des(:,i) = xd_des; 
    data.pos_abs_d(:,i) = vec4(DQ(xa_des).translation);
    data.pos_a(:,i) = vec4(DQ(xa).translation); 
    data.pos1_c(:,i) = vec4(DQ(xd_des).translation); 
    
    %relative pose
    data.pos_r(:,i) = vec4(DQ(xr).translation); 
    data.pos_r_d(:,i) = vec4(DQ(xr_des).translation); 

    %% Full-dual position dynamic control
    %%Define error:
   
    er = xd2_des - xr; %relative pose error
    ea = xd_des - xa;  %absolute pose error
    e = [er;ea]; 
    de = [dxd2_des-dxr;dxd_des - dxa];
    ei = e + cdt*de; 
    
    %display error each simulation step
    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_rel:',num2str(norm(er))])
    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_abs:',num2str(norm(ea))])

    data.norm(:,i) = norm(e);
    
    %%compute control input joint space
    Jaug = [Jr;Ja];
    Jaug_dot = [Jr_dot;Ja_dot];
    Jinv = pinv(Jaug);
    ddxd_aug_des = [ddxd2_des; ddxd_des]; 
    aq = Jinv*( ddxd_aug_des + kd*eye(16)*de + kp*eye(16)*e + ki*eye(16)*ei - Jaug_dot*qm_dot); 

    %%fb linearization
    tau = M*aq + c + g ;
    
   %% Null-space controllers
   
   P1 = eye(7) - pinv(J1)*J1; %null-space projector
   D_joints = eye(7)*2;
   K_joints = eye(7)*10; 
   tau_null1 = P1*(-D_joints*qm_dot1+ K_joints*(qc-qm1));
   P2 = eye(7) - pinv(J2)*J2; %null-space projector
   tau_null2 = P2*(-D_joints*qm_dot2 + K_joints*(qc-qm2));
  

    %% Torque command 
  
    tau1 = tau(1:7) + tau_null1;
    tau2 = tau(8:14) + tau_null2;  
    
    %Store sent torque commands for later
    data.tau1_send(:,i) = tau1;
    data.tau2_send(:,i) = tau2;

    %% Send torques to robots

    %Panda1
    for j=1:7
        if tau1(j)<0
            set_vel1 = -99999;
        else
            set_vel1 = 99999;
        end

        sim.simxSetJointTargetVelocity(clientID,joint_handles1(j),set_vel1,sim.simx_opmode_blocking);
        sim.simxSetJointForce(clientID,joint_handles1(j),abs(tau1(j)),sim.simx_opmode_blocking);
        [~,tau_read1] = sim.simxGetJointForce(clientID,joint_handles1(j),sim.simx_opmode_blocking);
        tau_read_data1(:,j) = tau_read1;
        
        if tau2(j)<0
            set_vel2 = -99999;
        else
            set_vel2 = 99999;
        end

        sim.simxSetJointTargetVelocity(clientID,joint_handles2(j),set_vel2,sim.simx_opmode_blocking);
        sim.simxSetJointForce(clientID,joint_handles2(j),abs(tau2(j)),sim.simx_opmode_blocking);
        [~,tau_read2] = sim.simxGetJointForce(clientID,joint_handles2(j),sim.simx_opmode_blocking);
        tau_read_data2(:,j) = tau_read2;
    end
    
    data.tau1_read(i,:) = tau_read_data1';

    %Panda2
    for j=1:7
        if tau2(j)<0
            set_vel = -99999;
        else
            set_vel = 99999;
        end

        sim.simxSetJointTargetVelocity(clientID,joint_handles2(j),set_vel,sim.simx_opmode_blocking);
        sim.simxSetJointForce(clientID,joint_handles2(j),abs(tau2(j)),sim.simx_opmode_blocking);
        [~,tau_read2] = sim.simxGetJointForce(clientID,joint_handles2(j),sim.simx_opmode_blocking);
        tau_read_data2(:,j) = tau_read2;

    end
    data.tau2_read(i,:) = tau_read_data2';
    
    time_check = sim.simxGetLastCmdTime(clientID)

    %---------------------------------
    sim.simxSynchronousTrigger(clientID);
    %---------------------------------
    i = i+1;
end

% Now close the connection to V-REP:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(clientID);

sim.delete();


%% PLOT DATA
%% Controller torques commands
f = figure; 
f.Renderer = 'painters';
f; 
plot(tt,data.tau1_send(1,:),'m--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,1),'m','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(2,:),'b--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(4,:),'k--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(5,:),'r--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,data.tau1_send(7,:),'y--','LineWidth',3);
hold on, grid on
plot(tt,data.tau1_read(:,7),'y','LineWidth',2);
legend('tsend','tread');

f2 = figure; 
f2.Renderer = 'painters';
f2; 
plot(tt,data.tau2_send(1,:),'m--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,1),'m','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(2,:),'b--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(4,:),'k--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(5,:),'r--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,data.tau2_send(7,:),'y--','LineWidth',3);
hold on, grid on
plot(tt,data.tau2_read(:,7),'y','LineWidth',2);
legend('tsend','tread');

%% Absolute position
f3 = figure; 
f3.Renderer = 'painters';
f3; 

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(2,:),'b-.','LineWidth',2.5);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Absolute position')

subplot(3, 1, 2)
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(3,:),'b-.','LineWidth',2.5);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 3)
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos1_c(4,:),'b-.','LineWidth',2.5);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','comp','Interpreter', 'latex', 'FontSize', 10)



%%Relative position
f4 = figure; 
f4.Renderer = 'painters';
f4; 

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_r_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(2,:),'c','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Relative position')

subplot(3, 1, 2)
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 3)
plot(tt,data.pos_r_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(4,:),'c','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','Interpreter', 'latex', 'FontSize', 10)



%% Ext forces
f5 = figure; 
f5.Renderer = 'painters';
f5; 

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.f1_ext(:,1),'LineWidth',2);
ylabel('$fx/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('External force Arm1')
subplot(3, 1, 2)
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
ylabel('$fy/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
grid on
hold on
plot(tt,data.f1_ext(:,3),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$fz/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

f6 = figure; 
f6.Renderer = 'painters';
f6; 

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.f2_ext(:,1),'LineWidth',2);
ylabel('$fx/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('External force Arm2')
subplot(3, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
ylabel('$fy/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
grid on
subplot(3, 1, 3)
grid on
hold on
plot(tt,data.f2_ext(:,3),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$fz/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


%% Arms EE positions
f7 = figure; 
f7.Renderer = 'painters';
f7; 

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos1(2,:),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Panda1')
subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos1(3,:),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos1(4,:),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


f8 = figure; 
f8.Renderer = 'painters';
f8; 
subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos2(2,:),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('Panda2')
subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos2(3,:),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos2(4,:),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)



