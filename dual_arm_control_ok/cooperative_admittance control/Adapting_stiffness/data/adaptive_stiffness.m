%% Adaptive stiffness plots
load("data/stiff.mat");
load("data/k_increase.mat"); 


%% Internal stresses comparison
mu = 0.3; %friction coefficient 


f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
title('Internal stresses')
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


%% gravity
f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
title('Weight forces')
grid on
hold on
plot(tt,data.f1_ext(:,3),'LineWidth',2);
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,3),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Internal stresses comparison with desired friction

f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,data.f1_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f1_ext(:,3))*mu*-1,'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2,4])
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data.f2_ext(:,2),'LineWidth',2);
hold on, grid on
plot(tt,(data.f2_ext(:,3))*mu,'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([2,4])
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


%% Relative pose
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
plot(tt,data.pos2_c(2,:),'b','LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(3, 1, 2)
grid on
hold on
plot(tt,data.pos_r_d(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos2_c(3,:),'b','LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(3, 1, 3)
grid on
hold on
plot(tt,data.pos_r_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data.pos_r(4,:),'c','LineWidth',2);
hold on, grid on
plot(tt,data.pos2_c(4,:),'b','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','comp','Interpreter', 'latex', 'FontSize', 10)

%% Adaptive stiffnes values
f1 = figure;
f1.Renderer = 'painters';

% subplot(3, 1, 1)
% grid on
% hold on
% plot(tt,kx_data,'b','LineWidth',2); 
% ylabel('$kx/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

 
% subplot(3, 1, 2)
% grid on
% hold on
% plot(tt,ky_data,'b','LineWidth',2); 
% ylabel('$ky/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
% 
%  
% subplot(3, 1, 3)
grid on
hold on
plot(tt,kz_data,'b','LineWidth',2); 
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$kz/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

