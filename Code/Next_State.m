function [next_state] = Next_State(curr,cont,mul,del_t)
% Venkatesh Prasad Venkataramanan
% PID : A53318036

% Calculating Twist %
vb = mul*(cont(6:9).');
del_wxy = vb*del_t;
del_wxy = del_wxy';

% Calculating next state %
next_state(1:3) = curr(1:3) + del_wxy;
next_state(4:8) = curr(4:8) + cont(1:5)*del_t;
next_state(9:12) = curr(9:12) + cont(6:9)*del_t;
end

