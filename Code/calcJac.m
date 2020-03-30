function [Je_pinv] = calcJac(thetalist,M0e,Tb0,mul)
% Venkatesh Prasad Venkataramanan
% PID : A53318036
Blist = [[0; 0; 1;   0; 0.033; 0], ...
       [0; -1; 0;   -0.5076;   0;   0], ...
       [0; -1; 0;   -0.3526;   0;   0], ...
       [0; -1; 0; -0.2176; 0; 0], ...
       [0; 0; 1; 0; 0; 0]];
%thetalist = [0; 0; 0.2; -1.6; 0];
J_arm = JacobianBody(Blist, thetalist);
%%%%%%%%%%%%%%%%%%%%%%
T0e = FKinBody(M0e, Blist, thetalist);

% Calculate J-base %
T0e_inv = TransInv(T0e);
Tb0_inv = TransInv(Tb0);
pro = T0e_inv * Tb0_inv;
J_base = Adjoint(pro)*mul;
%%%%%%%%%%%%%%%%%%%%%%%%

Je = [J_base J_arm];
% Calculating Pseudo Inverse %
Je_pinv = pinv(Je,1e-3);
end

