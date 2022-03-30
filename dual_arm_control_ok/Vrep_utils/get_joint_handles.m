%***************************************************************************
%                           get_joint_handles.m -  description
%                           ----------------------------
%    begin                : July 2020
%    authors              : Riddhiman Laha, Luis F.C. Figueredo
%    copyright            : (C) 2021 Technical University of Munich
%    email                : riddhiman <dot> laha <at> tum <dot> de
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% **************************************************************************
% Purpose
% --------------------------------------------------------------------------
% Function to obtain handles for different joints for VREP 
% 
% **************************************************************************
% Inputs: DQ_VrepInterface, clientID
% Output: handles
function handles = get_joint_handles(vi,id)

robot1_name = 'Franka1';
robot2_name = 'Franka2';
handles = struct('id',id);

%% arm joints
armJoints1 = -ones(1,7);
for i=1:7
    [res,armJoints1(i)] = vi.vrep.simxGetObjectHandle(id,[robot1_name,'_joint',num2str(i)],vi.vrep.simx_opmode_oneshot_wait);

end
handles.armJoints1 = armJoints1;

armJoints2 = -ones(1,7);
for i=1:7
    [res,armJoints2(i)] = vi.vrep.simxGetObjectHandle(id,[robot2_name,'_joint',num2str(i)],vi.vrep.simx_opmode_oneshot_wait);

end
handles.armJoints2 = armJoints2;

%% streaming
for i=1:7
    vi.vrep.simxGetJointPosition(id,armJoints1(i),vi.vrep.simx_opmode_streaming);

    vi.vrep.simxGetObjectFloatParameter(id,armJoints1(i),2012,vi.vrep.simx_opmode_streaming);

end

for i=1:7
    vi.vrep.simxGetJointPosition(id,armJoints2(i),vi.vrep.simx_opmode_streaming);

    vi.vrep.simxGetObjectFloatParameter(id,armJoints2(i),2012,vi.vrep.simx_opmode_streaming);

end

% Make sure that all streaming data has reached the client at least once
vi.vrep.simxGetPingTime(id);
end