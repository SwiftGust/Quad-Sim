%% Simple Saturation Function
% for Sliding Mode Control to prevent chattering
function [u] = sat(u)
    for i = 1:length(u)
    if u(i) > 1
        u(i) = 1;
    elseif u(i) < -1
        u(i) = -1;
    
    elseif   abs(u(i)) <= 1
        u(i) = u(i);
    end
end