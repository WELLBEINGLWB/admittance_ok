%***************************************************************************
%                           geomJ.m -  description
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
% Function to compute the Geometric Jacobian
% 
% **************************************************************************
% Inputs: dq_kinematics and joint configuration
% Output: 6xn geometric jacobian 
function [J] = geomJ(kine,q)
%
C8 = diag([-1 ones(1,3) -1 ones(1,3)]');
C4m = -C8(1:4,1:4);  
CJ4_2_J3= [zeros(3,1) eye(3)];
    
if strcmpi(class(kine), 'DQ_SerialManipulator') 
    Jacob = kine.pose_jacobian(q);
%     J = kuka.jacobian(q);
    xm = kine.fkm(q);
else
    Jacob = kine;
    xm = q;
end
    J(1:3,:) = CJ4_2_J3*2*haminus4(xm.P')*Jacob(1:4,:); 
    J(4:6,:) = CJ4_2_J3*2*( hamiplus4(xm.D)*C4m*Jacob(1:4,:) +  haminus4(xm.P')*Jacob(5:8,:));
    
end

