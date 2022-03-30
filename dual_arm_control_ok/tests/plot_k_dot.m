
ki = 300; %initial stiffness value
k_min = 1;
t = time;

k_temp_data = zeros(size(t,2),1);

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
    k_temp_data(i,1) = ki;
    i = i+1;
end

figure();
plot(t,k_temp_data,'Linewidth',2,'Color','r')
ylim([-10 400])

[k_d,d_d] = mod_k(time,k_in)
figure();
plot(t,k_d(:,:),'Linewidth',2,'Color','b')
ylim([-10 400])

%% Plot k_dot = f(k)
% syms k
% k_dot = 2*(k*alpha - 2*sqrt(k^3));
% 
% figure()
% fplot(k, k_dot)

%% Plot k_dot = f(t)

% ki = 300; %initial stiffness value
% k_min = 30; 
% mass = 1;
% alpha = 10*0.5*0.5;
% cdt = 0.01; %sampling time
% 
% 
% t = 0:cdt:2.5;
% k_temp_data = zeros(size(t,2),1);
% 
% i = 1;
% for i = 1:size(t,2)
%     if i~=1
%         k_dot = 2*(ki*alpha - 2*sqrt(ki^3));
%         k_temp = ki + k_dot*(t(i)-t(i-1));
%         if k_temp <= k_min
%             ki = k_min;
%         else
%             ki = k_temp;
%         end
%     else
%         ki = 300;
%         k_dot = 2*(ki*alpha - 2*sqrt(ki^3));
%         k_temp = ki + k_dot*t(i);
%         ki = k_temp;
%     end
% 
%     k_temp_data(i,1) = k_temp;
% 
%     i = i+1;
% end
% 
% 
% plot(t,k_temp_data)
% 
% kd_data = zeros(size(time,2),1); 
% 
% j = 1;
% for j = 1:size(t,2)
%     if j~=1
%         ki = kd_data(j-1,:);
%     else
%         ki = 300;
%     end
% 
%     [k_d,d_d] = imp_var(time(j),ki); 
%     
%     kd_data(j,:) = k_d; 
% 
%     j = j+1;
% end
% 
% figure();
% plot(tt,kd_data,'Linewidth',2,'Color','b')
