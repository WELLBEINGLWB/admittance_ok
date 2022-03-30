%% Robust pseudo Inverse
%% Compute pseudo-inverse of rectangular matrix robust to singularities (Levenberg-Marquardt algoritm) 

function J_inv = getpinv(J,mu)
s = size(J,1);
J_inv = J'*pinv(J*J' + mu*eye(s) );
end
