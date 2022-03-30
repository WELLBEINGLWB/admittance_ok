%% Dual Arm control
include_namespace_dq;

%% Initialize variables
%%admittance controller
xc_data = zeros(size(time,2),8);
dxc_data = zeros(size(time,2),8);
ddxc_data = zeros(size(time,2),8);
yr_data = zeros(size(time,2),6);
dyr_data =  zeros(size(time,2),6);

%%wrench vector
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

r0 = vec4(xa_in.P); 

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
for j=1:7
    [~,qmread1(j)]  = sim.simxGetJointPosition(clientID,joint_handles1(j),sim.simx_opmode_blocking);
end

qm1 = double([qmread1])';


for j=1:7
    [~,qmread2(j)]  = sim.simxGetJointPosition(clientID,joint_handles2(j),sim.simx_opmode_blocking);
end

qm2 = double([qmread2])'; 

qm = [qm1;qm2];

%% Saving data
data2.qm = [];  data2.qm_dot = []; data2.tau1_send = []; data2.tau1_read = [];
data2.tau2_send = []; data2.tau2_read = []; data2.pos_c = []; data2.f_ext = [];
data2.xr_des = [];  data2.xr = [];  data2.xa = []; data2.pos_a = [];  data2.pos_abs_d = [];
data2.x1 = []; data2.x2 = []; data2.norm = []; data2.pos_r = []; data2.pos_r_d = []; 

% time
inittime = sim.simxGetLastCmdTime(clientID)

%% Control loop

while sim.simxGetConnectionId(clientID)~=-1

    if i>size(time,2)
        break
    end

    % Getting current joint position and velocities
    %Panda1

    for j=1:7
        [~,qmread1(j)]  = sim.simxGetJointPosition(clientID,joint_handles1(j),sim.simx_opmode_blocking);
    end

    qmOld1 = qm1;
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

%     %Current relative and absolute pose
    xr = vec8(panda_bimanual.relative_pose(qm));
    xa = vec8(panda_bimanual.absolute_pose(qm));

    %Get jacobians
    J1 = fep1.pose_jacobian(qm1);
    J2 = fep2.pose_jacobian(qm2);

    dx1 = J1*qm_dot1;
    dx2 = J2*qm_dot2;
    
    %Jacobian derivatives
    J1_dot = fep1.pose_jacobian_derivative(qm1,qm_dot1);
    J2_dot = fep2.pose_jacobian_derivative(qm2,qm_dot2);
    
    %Store data2
    data2.qm(:,i) = qm;  data2.qm_dot(:,i) = qm_dot;  data2.xr(:,i) = xr; 
    data2.x1(:,i) = x1; data2.x2(:,i) = x2; data2.xr_des(:,i) = xr_des; data2.xa(:,i) = xa; 
    data2.pos_r(:,i) = vec4(DQ(xr).translation); 
    data2.pos_r_d(:,i) = vec4(DQ(xr_des).translation);
     
    data2.norm(:,i) = norm(xr_des-xr);
    
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
    % initialize variable
    if i~=1
        xr = xc_data(i-1,:)';
        yr_in = yr_data(i-1,:)';
        dyr_in = dyr_data(i-1,:)';
    else
        xr = vec8(xa_in);
        e_in = vec8(DQ(xr)'*DQ(xa_d(1,:)));
        yr_in = vec6(log(DQ(e_in)));
        dyr_in = zeros(6,1);
    end

    %External wrench
    switch fuse
        case 1
            psi_ext = zeros(1,6); % (wrt compliant frame)
            psi_ext_data(i,:) = psi_ext;
        case 2
            wrench_ext = ext_forces(xa);
            w_ext_data(i,:) = wrench_ext;
            psi_ext = vec6(DQ(r0)'*DQ(wrench_ext)*DQ(r0)); %external wrench (compliant frame)
            psi_ext = psi_ext';
            psi_ext_data(i,:) = psi_ext;
    end
    
    %ext forces world frame
    data2.f_ext(i,:) = w_ext_data(i,1:3)';   

    
    %Desired absolute pose variables
    xa_des = xa_d(i,:)'; 
    dxa_des = dxa_d(i,:)';
    ddxa_des = ddxa_d(i,:)';
    
    %Compute compliant trajectory
    [xd,dxd,ddxd,yr,dyr] = admittance_control(xa_des,dxa_des,ddxa_des,psi_ext,xr,yr_in,dyr_in,Md1,Kd1,Bd1,time);
    
    xc_data(i,:) = xd; 
    dxc_data(i,:) = dxd;
    ddxc_data(i,:) = ddxd;
    yr_data(i,:) = yr; 
    dyr_data(i,:) = dyr; 

    xd_des = xc_data(i,:)';
    dxd_des = dxc_data(i,:)';
    ddxd_des = ddxc_data(i,:)'; 

    data2.pos_abs_d(:,i) = vec4(DQ(xa_des).translation);
    data2.pos_a(:,i) = vec4(DQ(xa).translation); 
    data2.pos_c(:,i) = vec4(DQ(xd_des).translation); 

    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_rel:',num2str(norm(xr_des-xr))])
    disp(['it: ',num2str(i),' time(',num2str(i*cdt),') - err_pose_abs:',num2str(norm(xd_des-xa))])



    %% Motion control
    %%Assing desired EE poses for each arm in terms of CDTS variable
    %%(tracking xa and fixed xr).
  
    x2_des_conj = vec8(exp(0.5*log(DQ(xr_des)))*DQ(xd_des)'); 
    x2_des = DQ.C8*x2_des_conj; 
    dx2_des = vec8((DQ(xr_des)^0.5)*DQ(dxd_des));
    ddx2_des = vec8((DQ(xr_des)^0.5)*DQ(ddxd_des));

    x1_des = haminus8(DQ(xr_des))*x2_des; 
    dx1_des =  haminus8(DQ(xr_des))*dx2_des; 
    ddx1_des =  haminus8(DQ(xr_des))*ddx2_des; 
     
    %% Panda 1

    e1 = x1_des - x1;
    e_dot1 = dx1_des - dx1; 
    ei1 = e_dot1*cdt + e1;
    
    %%define desired closed-loop dynamics
    y1 =  kd*eye(8)*e_dot1 + kp*eye(8)*e1 + ki*eye(8)*ei1;
    
    %%control input task space
    ax1 = ddx1_des + y1 - J1_dot*qm_dot1;
    
    %%control input joint space
    aq1 = pinv(J1)*ax1;

    %%fb linearization
    tau1 = M1*aq1 + c1 + g1 ;
    

    %% Null-space controller
    N1 = haminus8(DQ(x1_des))*DQ.C8*J1;
    P1 = eye(7) - pinv(N1)*N1; %null-space projector
    D_joints = eye(7)*2;
    tau_null1 = P1*(-D_joints*qm_dot1);
 
    %% Torque command panda 1
    tau1 = tau1 + tau_null1;
    
    %% PANDA 2

    e2 = x2_des - x2;
    e_dot2 = dx2_des - dx2; 
    ei2 = e_dot2*cdt + e2;
    
    %%define desired closed-loop dynamics
    y2 =  kd*eye(8)*e_dot2 + kp*eye(8)*e2 + ki*eye(8)*ei2;
    
    %%control input task space
    ax2 = ddx2_des + y2 - J2_dot*qm_dot2;
    
    %%control input joint space
    aq2 = pinv(J2)*ax2;

    %%fb linearization
    tau2 = M2*aq2 + c2 + g2;
    

    %% Null-space controller
    N2 = haminus8(DQ(x2_des))*DQ.C8*J2;
    P2 = eye(7) - pinv(N2)*N2; %null-space projector
    D_joints = eye(7)*2;
    tau_null2 = P2*(-D_joints*qm_dot2);
 
    %% Torque command
    tau2 = tau2 + tau_null2;
    
    %Store sent torque commands for later
    data2.tau1_send(:,i) = tau1;
    data2.tau2_send(:,i) = tau2;

    %%Send torques to robots

    %Panda1
    for j=1:7
        if tau1(j)<0
            set_vel = -99999;
        else
            set_vel = 99999;
        end

        sim.simxSetJointTargetVelocity(clientID,joint_handles1(j),set_vel,sim.simx_opmode_blocking);
        sim.simxSetJointForce(clientID,joint_handles1(j),abs(tau1(j)),sim.simx_opmode_blocking);
        [~,tau_read1] = sim.simxGetJointForce(clientID,joint_handles1(j),sim.simx_opmode_blocking);
        tau_read_data1(:,j) = tau_read1;

    end
    data2.tau1_read(i,:) = tau_read_data1';

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
    data2.tau2_read(i,:) = tau_read_data2';
    
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
plot(tt,data2.tau1_send(1,:),'m--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,1),'m','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(2,:),'b--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(4,:),'k--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(5,:),'r--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.tau1_send(7,:),'y--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau1_read(:,7),'y','LineWidth',2);
legend('tsend','tread');

figure();
plot(tt,data2.tau2_send(1,:),'m--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,1),'m','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(2,:),'b--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,2),'b','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(3,:),'g--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,3),'g','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(4,:),'k--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,4),'k','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(5,:),'r--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,5),'r','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(6,:),'c--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,6),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.tau2_send(7,:),'y--','LineWidth',3);
hold on, grid on
plot(tt,data2.tau2_read(:,7),'y','LineWidth',2);
legend('tsend','tread');

%% EE position comparison
%%Absolute position
figure;
nexttile
plot(tt,data2.pos_abs_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_c(2,:),'b-.','LineWidth',2.5);
legend('xd','x','xc')
nexttile
plot(tt,data2.pos_abs_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_c(3,:),'b-.','LineWidth',2.5);
legend('yd','y','yc')
nexttile
plot(tt,data2.pos_abs_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_c(4,:),'b-.','LineWidth',2.5);
legend('zd','z','zc')
xlim([0,2.5])
xlabel('time [s]')
ylabel ('pos [m]')
title('Absolute position')

%Relative position
figure;
nexttile
plot(tt,data2.pos_r_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_r(2,:),'c','LineWidth',2);
legend('xd','x')
nexttile
plot(tt,data2.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_r(3,:),'c','LineWidth',2);
legend('yd','y')
nexttile
plot(tt,data2.pos_r_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data2.pos_r(4,:),'c','LineWidth',2);
legend('zd','z')
xlim([0,2.5])
xlabel('time [s]')
ylabel ('pos [m]')
title('Relative position')

%% Ext forces
figure()
plot(tt,data2.f_ext(:,1),'LineWidth',2);
hold on, grid on
plot(tt,data2.f_ext(:,2),'LineWidth',2);
hold on,grid on
plot(tt,data2.f_ext(:,3),'LineWidth',2);