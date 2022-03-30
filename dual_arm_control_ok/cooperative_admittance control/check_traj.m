%%Check traj
x1_des = zeros(size(time,2),8); 
x2_des = zeros(size(time,2),8); 
pos_a = zeros(size(time,2),4); 
pos_r = zeros(size(time,2),4); 
pos_1_d = zeros(size(time,2),4); 
pos_2_d = zeros(size(time,2),4); 


xa_in = normalize(DQ([-0.0001    0.7060    0.0001    0.7082   -0.0654   -0.0002    0.2848    0.0001]));
xr_in = normalize(DQ([-0.0698    0.9976    0.0000    0.0000   -0.0000   -0.0000    0.2783   -0.0000])); 


%[xad,dxad,ddxad,xrd,dxrd,ddxrd,grasp_data] = traj_gen(xa_in,xr_in,time);
% [xad,dxad,ddxad,xrd,dxrd,ddxrd,grasp_data] = grasp_traj_ad(xin_1,xin_2,time);
[xad,dxad,ddxad,xrd,dxrd,ddxrd] = gen_traj_lifting(xa_in,xr_in,time);


j = 1;
for j = 1:size(time,2)
    pos_a(j,:) = vec4(DQ(xad(j,:)).translation);
    pos_r(j,:) = vec4(DQ(xrd(j,:)).translation);
end

f6 = figure; 
f6.Renderer = 'painters';
f6; 
subplot(3, 1, 1)
plot(tt,pos_a(:,2),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('absolute pos')
subplot(3, 1, 2)
plot(tt,pos_a(:,3),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
plot(tt,pos_a(:,4),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

f6 = figure; 
f6.Renderer = 'painters';
f6; 
subplot(3, 1, 1)
plot(tt,pos_r(:,2),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('relative pos')
subplot(3, 1, 2)
plot(tt,pos_r(:,3),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
plot(tt,pos_r(:,4),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


%trajectory for each arm

j = 1;
for j = 1:size(time,2)
    x2_des(j,:) = DQ.C8*vec8(exp(0.5*log(DQ(xrd(j,:))))*DQ(xad(j,:))'); 
    x1_des(j,:) = haminus8(DQ(xrd(j,:)))*x2_des(j,:)';
    pos_1_d(j,:) = vec4(DQ(x1_des(j,:)).translation);
    pos_2_d(j,:) = vec4(DQ(x2_des(j,:)).translation);
    j = j+1;
end


f8 = figure; 
f8.Renderer = 'painters';
f8; 
subplot(3, 1, 1)
plot(tt,pos_1_d(:,2),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('arm1')
subplot(3, 1, 2)
plot(tt,pos_1_d(:,3),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
plot(tt,pos_1_d(:,4),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

f9 = figure; 
f9.Renderer = 'painters';
f9; 
subplot(3, 1, 1)
plot(tt,pos_2_d(:,2),'LineWidth',2);
ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
title('arm2')
subplot(3, 1, 2)
plot(tt,pos_2_d(:,3),'LineWidth',2);
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
subplot(3, 1, 3)
plot(tt,pos_2_d(:,4),'LineWidth',2);
xlabel('time [s]')
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

