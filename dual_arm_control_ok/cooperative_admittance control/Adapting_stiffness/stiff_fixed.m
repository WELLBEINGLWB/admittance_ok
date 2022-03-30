%% Stiff fixed

%% Stiffness adapter: modulate relative stiffness values to guarantee safe interaction and robustness to disturbance

function [K_var,D_var] = stiff_fixed(pos,x1,xr,delta,f1,f2,phase)

%% Description: Modulate stiffness and damping on each cartesian dof of the impedance controller to guarantee good
%%              tracking perfomance and safe interaction.

%%Inputs:     pos = current absolute position; [3x1]
%             delta = current position displacement [3x1]
%             x1 = current left arm pose [8x1]
%             f1 = measured force on EE1 [3x1]
%             f2 = measured force on EE2 [3x1]
%             phase = flag acknowledging the current desired task phase.

%%Outputs:    k = value of stiffness on single d.o.f;
%             d = value of damping on single d.o.f.

%%Parameters: F_max = treshold of ext disturbance
%             k_default = default value of stiffness
%             k_min = minimum value of stiffness
%             a0,berta,csi = parameters from stability conditions
%             mass = apparent mass

mass = 1.5; %kg
kr_default = 100; %N/m
kr_rot = 300;
dr_rot = 2*sqrt(mass*kr_rot); 
dr_default = 2*sqrt(mass*kr_default); %N*s/m
k_min = 10; %N/m lower bound for stiffness
z_table = 0.2942; %table position

%friction coefficient
mu = 0.4;
eta = 1/(sqrt(1+mu^2));


%retrieve frames orientation
r_r = DQ(xr).P; %relative frame orientation
r1 = DQ(x1).P; %arm1 orientation

%% Initialization
persistent krx kry krz drx dry drz
% persistent initialization
if isempty(krx)
    krx = kr_default;
end

if isempty(kry)
    kry = kr_default;
end
if isempty(krz)
    krz = kr_default;
end

if isempty(drx)
    drx = 2*sqrt(mass*krx);
end
if isempty(dry)
    dry = 2*sqrt(mass*kry);
end
if isempty(drz)
    drz = 2*sqrt(mass*krz);
end


%% Compute k
if phase == 1
    krx = kr_default;
    kry = kr_default;
    krz = kr_default;
    
    drx = 2*sqrt(mass*krx);
    dry = 2*sqrt(mass*kry);
    drz = 2*sqrt(mass*krz);

elseif phase ==2
    if (abs(f1(2)) || abs(f2(2)) > 0) %contact with obj is detected
        krx = kr_default;
        kry = kr_default; 
        krz = 0.995*krz; %empirically decrease k
        drx = 2*sqrt(mass*krx);
        dry = 2*sqrt(mass*kry);
        drz = 2*sqrt(mass*krz);
        % set lower bound for stiffness value
        if krx <= k_min
            krx = kmin;
            drx = 2*sqrt(mass*krx);
        end
        if kry <= k_min
            kry = kmin;
            dry = 2*sqrt(mass*kry);
        end
        if krz <= k_min
            krz = kmin;
            drz = 2*sqrt(mass*krz);
        end

    end
else
    if pos(3) > z_table + 0.005 %detaching from table

        %condition to avoid slipping
        f1_des = [0;mu*f1(3);0];
        f2_des = [0;-mu*f2(3);0];

        %express with respect to left arm
        f1_des = vec3(r1'*f1_des*r1);
        f2_des = vec3(r1'* f2_des*r1);

        %mapping to relative desired frame
        fr_d = 0.5*(f2_des - f1_des);
        Fr_des = vec3(r_r'*fr_d*r_r);

        %increase stiffness to compensate additional forces
        krx = kr_default;
        kry = kr_default;
        krz = abs(Fr_des(3)/delta(3));

        drx = 2*sqrt(mass*krx);
        dry = 2*sqrt(mass*kry);
        drz = 2*sqrt(mass*krz);

    end
end

k = [krx;kry;krz];
d = [drx;dry;drz]; 

K_var = diag([k;kr_rot*ones(3,1)]);
D_var = diag([d;dr_rot*ones(3,1)]);

end


