%% Dual Arm controller

%% Full-dual position control
%% Description: Impedance control on absolute pose, fixed relative pose
%% + task-space inverse dynamics motion controller for 12 dofs (relative + absolute pose).

include_namespace_dq;

%% Initialize variables
%%Admittance controller
xc_data = zeros(size(time,2),8);
dxc_data = zeros(size(time,2),8);
ddxc_data = zeros(size(time,2),8);
yr_data = zeros(size(time,2),6);
dyr_data =  zeros(size(time,2),6);

%%Wrench vector
w_ext_data = zeros(size(time,2),6); %external wrench on EE (world_frame)
psi_ext_data = zeros(size(time,2),6); %external wrench on EE (complinat_reference_frame)

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

%Build dual-arm system
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

%Desired relative pose
xr_des = vec8(xr_in); 

%Desired absolute pose trajectory
switch fuse
    case 1
        [xa_d,dxa_d,ddxa_d] = gen_traj(xa_in,time);
    case 2
        [xa_d,dxa_d,ddxa_d] = traj(xa_in,time);
end

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
data.qm = [];  data.qm_dot = []; data.tau1_send = []; data.tau1_read = [];
data.tau2_send = []; data.tau2_read = []; data.f_ext = [];
data.xr_des = [];  data.xr = [];  data.xa = []; data.xa_des = []; data.pos_c = []; 
data.x1 = []; data.x2 = []; data.norm = []; data.pos_a = [];  data.pos_abs_d = [];
data.pos_r = []; data.pos_r_d = []; data.kd = []; data.bd = []; 
    
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
    data.x1(:,i) = x1; data.x2(:,i) = x2; data.xr_des(:,i) = xr_des; data.xa(:,i) = xa; 
    data.pos_r(:,i) = vec4(DQ(xr).translation); data.pos_r_d(:,i) = vec4(DQ(xr_des).translation); 
    
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
        xr_adm = xc_data(i-1,:)';
        yr_in = yr_data(i-1,:)';
        dyr_in = dyr_data(i-1,:)';
    else
        xr_adm = vec8(xa_in);
        e_in = vec8(DQ(xr_adm)'*DQ(xa_d(1,:)));
        yr_in = vec6(log(DQ(e_in)));
        dyr_in = zeros(6,1);
    end
    
    switch fuse
        case 1
            %External wrench
            psi_ext = zeros(1,6); % (wrt compliant frame)
            psi_ext_data(i,:) = psi_ext;
        case 2
            %External wrench
            wrench_ext = ext_forces(xa);
            w_ext_data(i,:) = wrench_ext;
            psi_ext = vec6(DQ(r0)'*DQ(wrench_ext)*DQ(r0)); %external wrench (compliant frame)
            psi_ext = psi_ext';
            psi_ext_data(i,:) = psi_ext;
    end
    
    %ext forces world frame
    data.f_ext(i,:) = w_ext_data(i,1:3)';

    %Desired absolute pose variables
    xa_des = xa_d(i,:)'; 
    dxa_des = dxa_d(i,:)';
    ddxa_des = ddxa_d(i,:)';

    %Compute compliant trajectory
    [xd,dxd,ddxd,yr,dyr] = admittance_control(xa_des,dxa_des,ddxa_des,psi_ext,xr_adm,yr_in,dyr_in,Md1,Kd1,Bd1,time);
    
    xc_data(i,:) = xd; 
    dxc_data(i,:) = dxd;
    ddxc_data(i,:) = ddxd;
    yr_data(i,:) = yr; 
    dyr_data(i,:) = dyr; 

    xd_des = xc_data(i,:)';
    dxd_des = dxc_data(i,:)';
    ddxd_des = ddxc_data(i,:)'; 
    
    data.xa_des(:,i) = xd_des; 
    data.pos_abs_d(:,i) = vec4(DQ(xa_des).translation);
    data.pos_a(:,i) = vec4(DQ(xa).translation); 
    data.pos_c(:,i) = vec4(DQ(xd_des).translation); 

    %% Full-dual position dynamic control
    %%Define error:
    %%e = [xrd - xr; xa_d - xa]; %full dual position error
    
    er = xr_des - xr;
    ea = xd_des - xa;
    e = [er;ea]; 
    de = [-dxr;dxd_des - dxa];
    ei = e + cdt*de; 
    
    %display error each simulation step
    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_rel:',num2str(norm(er))])
    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_abs:',num2str(norm(ea))])

    data.norm(:,i) = norm(e);
    
    %%compute control input joint space
    Jaug = [Jr;Ja];
    Jaug_dot = [Jr_dot;Ja_dot];
    Jinv = pinv(Jaug);
    ddxd1_des = [zeros(8,1); ddxd_des]; 
    aq = Jinv*( ddxd1_des + kd*eye(16)*de + kp*eye(16)*e + ki*eye(16)*ei - Jaug_dot*qm_dot); 

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
figure();
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

figure();
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
figure;
nexttile
plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos_c(2,:),'b-.','LineWidth',2.5);
legend('xd','x','xc')
nexttile
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos_c(3,:),'b-.','LineWidth',2.5);
legend('yd','y','yc')
nexttile
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos_c(4,:),'b-.','LineWidth',2.5);
legend('zd','z','zc')
xlabel('time [s]')
ylabel ('pos [m]')
title('Absolute position')

%%Relative position
figure;
nexttile
plot(tt,data.pos_r_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(2,:),'c','LineWidth',2);
legend('xd','x')
nexttile
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
legend('yd','y')
nexttile
plot(tt,data.pos_r_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(4,:),'c','LineWidth',2);
legend('zd','z')
xlim([0,2.5])
xlabel('time [s]')
ylabel ('pos [m]')
title('Relative position')

%% Ext forces
figure()
plot(tt,data.f_ext(:,1),'LineWidth',2);
hold on, grid on
plot(tt,data.f_ext(:,2),'LineWidth',2);
hold on,grid on
plot(tt,data.f_ext(:,3),'LineWidth',2);
xlabel('time [s]')
ylabel ('force [N]')
