%% DUAL ARM DYNAMIC CONTROL PERFOMANCE ANALYSIS
%% Description: Comparison of 3 dynamic controllers perfomances for a panda dual-arm system.
%%fuse = 1 --> free-motion tracking task (y-z plane);
%%fuse = 2 --> model of an interaction detected on z axis. 

%% Load data (5s simulation)
tt = 0:0.01:5; 

if fuse == 1
    %%Free-motion task
    load("data_decoupled_controllers.mat");
    load("data_full_position_control.mat");
    load("data/data_multipriority.mat")
else
    %%Interaction task
    load("data_full_interaction.mat");
    load("data_decoupled_interaction.mat");
    load("data/data_multipriority_interaction.mat");
end

%% Compare reference and the measured value for the cartesian absolute position.
%% Absolute Position comparison

line_width = 2.5;
f = figure;
f.Renderer = 'painters';

%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_a(2,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(2,:),'g','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','full','decoupled','nullspace', 'Interpreter', 'latex', 'FontSize', 10)

subplot(3, 1, 2)
plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_a(3,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(3,:),'g','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 3)
plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_a(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_a(4,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_a(4,:),'g','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


%% Comparison reference and the measured value for the cartesian relative position.
%% Relative Position comparison

line_width = 2.5;
f1 = figure;
f1.Renderer = 'painters';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3, 1, 1)
grid on
hold on
plot(tt,data.pos_r_d(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_r(2,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(2,:),'g','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','full','decoupled','nullspace', 'Interpreter', 'latex', 'FontSize', 10)

subplot(3, 1, 2)
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_r(3,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(3,:),'g','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 3)
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data2.pos_r(3,:),'b','LineWidth',2);
hold on, grid on
plot(tt,data3.pos_r(3,:),'g','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

%% External forces comparison

f2 = figure();
f2.Renderer = 'painters';
plot(tt,data.f_ext(:,3),'LineWidth',2);
hold on, grid on
plot(tt,data2.f_ext(:,3),'LineWidth',2);
hold on, grid on
plot(tt,data3.f_ext(:,3),'Linewidth',2)
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$fz/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('full','decoupled','nullspace','Interpreter', 'latex', 'FontSize', 10)



%% SIMULATION PLOTS
% %% full-dual position control
% 
% %%Absolute position
% figure();
% tiledlayout(3,1)
% 
% nexttile
% plot(tt,data.pos_abs_d(2,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_a(2,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% xlabel('time [s]')
% ylabel('x [m]')
% title('Absolute position')
% legend('des','curr')
% 
% nexttile
% plot(tt,data.pos_abs_d(3,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_a(3,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% xlabel('time [s]')
% ylabel('y [m]')
% 
% nexttile
% plot(tt,data.pos_abs_d(4,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_a(4,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% hold on, grid on
% xlabel('time [s]')
% ylabel('z [m]')
% 
% %%Relative position
% figure();
% tiledlayout(3,1)
% 
% nexttile
% plot(tt,data.pos_r_d(2,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_r(2,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% xlabel('time [s]')
% ylabel('x [m]')
% title('Relative position')
% legend('des','curr')
% 
% nexttile
% plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_r(3,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% xlabel('time [s]')
% ylabel('y [m]')
% 
% nexttile
% plot(tt,data.pos_r_d(4,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data.pos_r(4,:),'Color',[0.2 0.4 0.6],'LineWidth',2);
% hold on, grid on
% xlabel('time [s]')
% ylabel('z [m]')
% 
% 
% %Ext force
% figure();
% title('External force')
% plot(tt,data.f_ext(:,3),'Color',[0.9 0.5 0],'LineWidth',2);
% grid on
% xlabel('time [s]')
% ylabel('F [N]')
% 
