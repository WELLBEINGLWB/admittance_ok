function [wr1,wr2] = wrench_ext(x1,x2,grasp)

%% Description: %% Model of external forces/moments exerted on the arms (world frame)
%% Assumption: elastic reaction of the environment/object

% Outputs: wr1 = external wrench on arm1 (world frame) [6x1]
%          wr2 = external wrench on arm1 (world frame) [6x1]
% Inputs:  x1 = current EE1 pose [8x1].
%          x2 = current EE2 pose [8x1].
%          grasp = flag acknowledging lifting phase


%Environment parameters
mu = 0.4; 
g = 9.81; %m/s^2
mass_obj = 0.5; %kg
k_table = 5000; %N/m, environment stiffness
k_obj = 500; %N/m, object stiffness
%
% l_obj = 0.2; %m length 
% p1 = -0.05; %contact position between arm 1 and object (y axis)
% p2 = 0.15; %contact position between arm 1 and object (y axis)
% pc = 0.275; %contact position with table (z axis)

%
l_obj = 0.1; %m length 
p1 = -0.05; %contact position between arm 1 and object (y axis)
p2 = 0.05; %contact position between arm 1 and object (y axis)
pc = 0.2942; %contact position with table (z axis)

%retrieve positions
x1_pos = vec4(DQ(x1).translation); %EE position
z1 = [x1_pos(2); x1_pos(3); x1_pos(4)];

x2_pos = vec4(DQ(x2).translation); %EE position
z2 = [x2_pos(2); x2_pos(3); x2_pos(4)];


%% Initialization
F1 = zeros(3,1);
M1 = zeros(3,1);
F2 = zeros(3,1);
M2 = zeros(3,1);

%%model
if grasp == 0
    if z1(2) > p1 %contact with the object
        F1(2) = -k_obj*(z1(2)-p1); %force to the left
    end
    if z2(2) < p2
        F2(2) = -k_obj*(z2(2)-p2); %force to the right
    end
else
    F1(2) = 0;
    F2(2) = 0;
    %still in contact/lift
    c1 = min(0,z1(3)-pc);
    c2 = min(0,z2(3)-pc);
    
    if c1 == 0
        %detached
        F1(2) = -mu*0.5*(mass_obj*g);
        F1(3) = -k_table*c1 + 0.5*(mass_obj*g);
    else
        %still in contact 
        F1(3) = -k_table*c1; 
    end

    if c2 == 0
        %detached
        F2(2) = mu*0.5*(mass_obj*g);
        F2(3) = -k_table*c2 + 0.5*(mass_obj*g);
    else
       %still in contact 
        F2(3) =-k_table*c2; 
    end

    M1(1) = F2(3)*l_obj;
    M2(1) = -F1(3)*l_obj;    
          
end

%Compute wrenches
wr1 = [F1;M1];
wr2 = [F2;M2];

end
