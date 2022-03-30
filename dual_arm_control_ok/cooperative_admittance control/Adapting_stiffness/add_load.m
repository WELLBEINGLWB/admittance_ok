%Model of additional load
function f_add = add_load(time,t_dist)
    add_weight = 5; %N
    m = add_weight/0.3; %angular coefficient
    f_add = 0 + m*(time - t_dist);
    if f_add >= add_weight
        f_add = add_weight;
    end
end
   