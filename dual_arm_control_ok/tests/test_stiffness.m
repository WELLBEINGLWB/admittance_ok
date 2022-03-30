%% test lower stiffness values

%% alpha = 10;
load("var_imp_full.mat");
load("var_imp_null_space.mat");


%% Compare reference and the measured value for the cartesian absolute position.
%% Position comparision

figure();
plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(2,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('x [m]')
legend('des','full','nullspace')
title('Absolute position')


figure();
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(3,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('y [m]')
legend('des','full','nullspace')
title('Absolute position')

figure();
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(4,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('z [m]')
legend('des','full','nullspace')
title('Absolute position')

%% Comparison reference and the measured value for the cartesian relative position.
%% Position comparision

figure();
plot(tt,data.pos_r_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(2,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('x [m]')
legend('des','full','nullspace')
title('Relative position')

figure();
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(3,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('y [m]')
legend('des','full','nullspace')
title('Relative position')


figure();
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(3,:),'g','LineWidth',2);
xlabel('time [s]')
ylabel('z [m]')
legend('des','full','nullspace')
title('Relative position')


%% External forces comparison
figure();
plot(tt,data.f_ext(:,3),'LineWidth',2);
hold on, grid on
plot(tt,data3.f_ext(:,3),'Linewidth',2)
legend('full','nullspace')
title('External forces')

%% Gains

figure();
plot(tt,data.kd(:,3),'LineWidth',2);
xlabel('time [s]')
ylabel('kz [N/m]')

figure();
plot(tt,data.bd(:,3),'LineWidth',2);
xlabel('time [s]')
ylabel('dz [Ns/m]')
