%% Wrench adapter
function [Wa,Wr] = wrench_mapping(wr_1,wr_2,x1,x2,xa)

%% Description: Linear mapping from the external wrenches perceived at the two EEs to the absolute and relative task-space
%%NB: (virtual sticks representation)

%%Outputs:  Wa,Wr = cdts absolute and relative wrenches; (world frame) 6x1
%%Inputs:   wr_1,wr_2 = mesured external wrench at the tip of each EE (world-frame);
%           x1, x2 = EEs poses;
%           xa = absolute pose 

%retrieve position

p1 = vec4(DQ(x1).translation);
p2 = vec4(DQ(x2).translation);
pa = vec4(DQ(xa).translation); 

%retrieve orientation of left arm
r1 = vec4(DQ(x1).P);

%virtual stick disp
p1_a = [pa(2) - p1(2); pa(3) - p1(3); pa(4) - p1(4)];
p2_a = [pa(2) - p2(2); pa(3) - p2(3); pa(4) - p2(4)];

%external forces wrt to left arm frame 
wr_1_1 = vec6(DQ(r1)'*wr_1*DQ(r1));
wr_2_1 = vec6(DQ(r1)'*wr_2*DQ(r1));

%absolute and relative task-space
fa = wr_1(1:3,:) + wr_2(1:3,:); %abs force
fr = 0.5*(wr_2_1(1:3,:) - wr_1_1(1:3,:)); %rel force
Ma = wr_1(4:6,:) + cross(p1_a,wr_1(1:3,:)) + wr_2(4:6,:) + cross(p2_a,wr_2(1:3,:)); %abs moment
Mr =  0.5*(wr_2_1(4:6,:) - wr_1_1(4:6,:)); %rel moment

%Wrenches
Wa = [fa;Ma]; 
Wr = [fr;Mr];

end
