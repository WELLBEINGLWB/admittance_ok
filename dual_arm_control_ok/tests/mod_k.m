function [k_d,d_d] = mod_k(time,ki)
%%Initialize
k_d = [zeros(size(time,2),1)];
d_d = [zeros(size(time,2),1)];

%%parameters
mass = 1; 
k_min = 10; %minimum stiffness value

%decreasing rate
k_dot = -1e4;
%simulation time
t = time; 

i = 1;
for i = 1:size(t,2)
    if i~=1
        k_dot = -1e4;
        k_temp = ki + k_dot*(t(i)-t(i-1));
        if k_temp <= k_min
            ki = k_min;
        else
            ki = k_temp;
        end
    else
        ki = 300;
        k_dot = -1e4;
        k_temp = ki + k_dot*t(i);
        ki = k_temp;
    end
    
    k_d(i,:) = ki;
    d_d(i,:) = 2*sqrt(ki); 

    i = i+1;
end
