%% no control on relative spring

load("data/tight_grasp.mat"); 

f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
title('Internal stresses')
grid on
hold on
plot(tt,data3.f1_ext(:,2),'LineWidth',2);
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2, 1, 2)
grid on
hold on
plot(tt,data3.f2_ext(:,2),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Relative pose
f1 = figure;
f1.Renderer = 'painters';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(3, 1, 1)
% grid on
% hold on
% plot(tt,data3.pos_r_d(2,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data3.pos_r(2,:),'c','LineWidth',2);
% ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
% 
%  
% subplot(3, 1, 2)
% grid on
% hold on
% plot(tt,data3.pos_r_d(3,:),'r--','LineWidth',3); 
% hold on, grid on
% plot(tt,data3.pos_r(3,:),'c','LineWidth',2);
% ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
% 
% 
% subplot(3, 1, 3)
grid on
hold on
plot(tt,data3.pos_r_d(4,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,data3.pos_r(4,:),'c','LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','curr','Interpreter', 'latex', 'FontSize', 10)