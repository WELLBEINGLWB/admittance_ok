function [xad,dxad,ddxad,xrd,dxrd,ddxrd,grasp_data] = traj_gen(xa_in,xr_in,time)

%% Description: generates minimum jerk task-space trajectory (DQ representation)
%%Inputs:  xa_in = initial absolute pose; [8x1]
%          xr_in = initial relative pose; [8x1]
%          time = simulation time
%%Outputs: xd,dxd,ddxd = desired end-effector pose and its 1st and 2nd time derivative.  [8x1]
%          grasp_data = flag acknowleding phases of grasping task

%%Initialize
%absolute pose trajectory
xad = [zeros(size(time,2),8)];
dxad = [zeros(size(time,2),8)];
ddxad = [zeros(size(time,2),8)];

%relative pose trajectory
xrd = [zeros(size(time,2),8)];
dxrd = [zeros(size(time,2),8)];
ddxrd = [zeros(size(time,2),8)];

grasp_data = [zeros(size(time,2),1)]; 

grasp = 0; 

%%retrieve initial conditions
%absolute pos/or
p0 = vec4(xa_in.translation);
r0 = vec4(P(xa_in));
pos_i = [p0(2);p0(3);p0(4)];

%relative pos/or
p0_r = vec4(xr_in.translation);
r0_r = vec4(P(xr_in));
pos_i_r = [p0_r(2);p0_r(3);p0_r(4)];

i = 1;  

for i = 1:size(time,2)
    if (time(i) >=0 && time(i) < 0.5) %go down
        pos_f = pos_i + [0;0;-0.15];
        pos_r_f = pos_i_r; %constant relative position
        tf = 0.5;
        t = time(i);
    elseif (time(i) >=0.5 && time(i) < 0.7) %pause
        pos_i = [p0(2);p0(3);p0(4)-0.15];
        pos_f = pos_i;
        pos_r_f = pos_i_r;
        tf = 0.2;
        t = time(i) - 0.5;
    elseif (time(i) >= 0.7 && time(i) < 1.7) %(approach obj)
        pos_i = [p0(2);p0(3);p0(4)-0.15];
        pos_f = pos_i + [0;0.05;-0.07]; 
        pos_r_f = pos_i_r + [0;0;-0.39]; 
        tf = 1;
        t = time(i) - 0.7;
    elseif (time(i) >= 1.7 && time(i) < 2) %(pause)
        pos_i = [p0(2);p0(3);p0(4)] + [0;0.05;-0.15-0.07];
        pos_f = pos_i; 
        pos_i_r = [p0_r(2);p0_r(3);p0_r(4)] + [0;0;-0.39];
        pos_r_f = pos_i_r ; 
        tf = 0.3;
        t = time(i) - 1.7;
    elseif (time(i) >= 2 && time(i) < 2.4) %go up
        pos_i = [p0(2);p0(3)+0.05;p0(4)-0.15-0.07];
        pos_f = pos_i + [0;0;0.15+0.07]; 
        pos_i_r = [p0_r(2);p0_r(3);p0_r(4)] + [0;0;-0.39]; 
        pos_r_f = pos_i_r; 
        tf = 0.4;
        t = time(i) - 2;
        grasp = 1;
    else
        pos_i = [p0(2);p0(3)+0.05;p0(4)];
        pos_f = pos_i;
        pos_i_r = [p0_r(2);p0_r(3);p0_r(4)] + [0;0;-0.39]; 
        pos_r_f = pos_i_r; 
        tf = 1000;
        t = time(i) - 2.2;
        grasp = 1; 
    end

    %% Minimum jerk interpolation
    %abs pose
    zd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    %rel pose
    zr_d = pos_i_r + (pos_i_r - pos_r_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzr_d = (pos_i_r - pos_r_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzr_d = (pos_i_r - pos_r_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    
    %% Compute trajectory
    %abs pose
    x_des = vec8(DQ(r0) + 0.5*DQ.E*(DQ(zd)*DQ(r0)));
    dx_des =  vec8(0.5*DQ.E*(DQ(dzd)*DQ(r0)));
    ddx_des = vec8(0.5*DQ.E*(DQ(ddzd)*DQ(r0)));
 
    %rel pose
    xr_des = vec8(DQ(r0_r) + 0.5*DQ.E*(DQ(zr_d)*DQ(r0_r)));
    dxr_des =  vec8(0.5*DQ.E*(DQ(dzr_d)*DQ(r0_r)));
    ddxr_des = vec8(0.5*DQ.E*(DQ(ddzr_d)*DQ(r0_r)));

    
    xad(i,:) = x_des;
    dxad(i,:) = dx_des;
    ddxad(i,:) = ddx_des;
    
    xrd(i,:) = xr_des;
    dxrd(i,:) = dxr_des;
    ddxrd(i,:) = ddxr_des;


    grasp_data(i,:) = grasp; 
    
    i = i+1;
end

end


