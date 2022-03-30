
function [xd,dxd,ddxd] = gen_traj(x_in,time)

%% Description: generates minimum jerk task-space trajectory (DQ representation)
%%Inputs:  x_in = initial end-effector pose [8x1]
%          time = simulation time
%%Outputs: xd,dxd,ddxd = desired end-effector pose and its 1st and 2nd time derivative.  [8x1]

%%initialize
xd = [zeros(size(time,2),8)];
dxd = [zeros(size(time,2),8)];
ddxd = [zeros(size(time,2),8)];

%%retrieve initial conditions
p0 = vec4(x_in.translation);
r0 = vec4(P(x_in));
pos_i = [p0(2);p0(3);p0(4)];
i = 1;  

for i = 1:size(time,2)
    if (time(i)>=0 && time(i)<1) %go down
        pos_f = pos_i + [0;0;-0.2];
        tf = 1;
        t = time(i);
    elseif (time(i)>= 1 && time(i)< 1.3) %pause
        pos_i = [p0(2);p0(3);p0(4)-0.2];
        pos_f = pos_i;
        tf = 0.3;
        t = time(i) - 1;
    elseif (time(i)>=1.3 && time(i)<2.3) %go +y
        pos_i = [p0(2);p0(3);p0(4)-0.2];
        pos_f = pos_i + [0;0.1;0];
        tf = 1;
        t = time(i) - 1.3;
    elseif (time(i)>=2.3 && time(i)<2.6) %pause
        pos_i = [p0(2);p0(3)+0.1;p0(4)-0.2];
        pos_f = pos_i;
        tf = 0.3;
        t = time(i) - 2.3;
    elseif (time(i)>=2.6 && time(i)<4) %go up and backwards
        pos_i = [p0(2);p0(3)+0.1;p0(4)-0.2];
        pos_f = pos_i + [0;0;+0.2];
        tf = 1.4;
        t = time(i)-2.6;
    else
        pos_i = [p0(2);p0(3)+0.1;p0(4)];
        pos_f = pos_i;
        tf = 1000;
        t = time(i) - 4;
    end
    
    %% Minimum jerk interpolation
    zd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    
    x_des = vec8(DQ(r0) + 0.5*DQ.E*(DQ(zd)*DQ(r0)));
    dx_des =  vec8(0.5*DQ.E*(DQ(dzd)*DQ(r0)));
    ddx_des = vec8(0.5*DQ.E*(DQ(ddzd)*DQ(r0)));
    
    xd(i,:) = x_des;
    dxd(i,:) = dx_des;
    ddxd(i,:) = ddx_des;
    i = i+1;
end

end



