function [wr1,wr2] = wrench_ext_grasp_task(x1,x2,grasp,time,fuse)

%% Description: %% Model of external forces/moments exerted on the arms (world frame)
%% Assumption: elastic reaction of the environment/object

% Outputs: wr1 = external wrench on arm1 (world frame) [6x1]
%          wr2 = external wrench on arm1 (world frame) [6x1]
% Inputs:  x1 = current EE1 pose [8x1].
%          x2 = current EE2 pose [8x1].
%          grasp = flag acknowledging lifting phase


%Environment parameters
g = 9.81; %m/s^2
mass_obj = 0.5; %kg
k_table = 5000; %N/m, environment stiffness
k_obj = 500; %N/m, object stiffness
%
l_obj = 0.08; %m length 
p1 = -0.05; %contact position between arm 1 and object (y axis)
p2 = 0.05; %contact position between arm 1 and object (y axis)
pc = 0.2942; %contact position with table (z axis)

%friction coefficient
mu_nom = 0.4;
alpha = 0.2; %uncertainty of 20%
% mu_wc = mu_nom*(1-alpha); %worst case 

%%Added load
t_dist = 2.5; %additionaln weight instant of time 

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
if time >= t_dist %%additional weight added
    sw = 1;
else
    sw = 0;
end


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

    %compressing
    d1 = max(0,z1(2)-p1);
    d2 = min(0,z2(2)-p2);    

    if c1 == 0 %detached        
        F1(3) = -k_table*c1 + 0.5*(mass_obj*g);
        f_add = add_load(time,t_dist); 
        F1(3) = 0.5*mass_obj*g + 0.5*f_add*sw; 

%         if d1 == 0 %not in contact
%            F1(2) = 0*-mu_nom*F1(3); %friction due to weight
%         else %tighteting 
%            F1(2) = -k_obj*d1 -mu_nom*F1(3); %friction + squeezing
%         end     
        if d1 == 0 %not in contact
           F1(2) = 0*-mu_nom*F1(3); %friction due to weight
        else
            if (abs(k_obj*d1) < abs(mu_nom*F1(3))) % tighting
                F1(2) = -(k_obj)*d1; %
            else
                F1(2) = -(k_obj)*d1 ; %friction + squeezing 
            end
        end 

    else
        %still in contact 
        F1(3) = -k_table*c1; 
    end

    if c2 == 0  %detached
        F2(3) = -k_table*c2 + 0.5*(mass_obj*g);
        
        if time >= t_dist
            sw = 1;
        else
            sw = 0;
        end
        f_add = add_load(time,t_dist);
        F2(3) = 0.5*(mass_obj*g) + 0.5*f_add*sw;

%         if d2 == 0 %not squeezing
%            F2(2) = 0*mu_nom*F2(3); %friction
%         else %tighteting 
%            F2(2) = -k_obj*d2 + mu_nom*F2(3);
%         end
        if d2 == 0 %not in contact
           F2(2) = 0*-mu_nom*F1(3); %friction due to weight
        else
            if (abs(k_obj*d2) < abs(mu_nom*F2(3))) % tighting
                F2(2) = -(k_obj)*d2; %
            else
                F2(2) = -(k_obj)*d2 ; %friction + squeezing 
            end
        end 

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


