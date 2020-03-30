function [vb,Xerr] = feedbackcontrol(X,Xd,Xd_next,Kp,Ki,del_t)
% Venkatesh Prasad Venkataramanan
% PID : A53318036

% Pretty straightforward %
X_inv = TransInv(X);
adjoint = Adjoint(X_inv*Xd);

X_d_inv = TransInv(Xd);
pro = X_d_inv*Xd_next;

vd_se3 = (1/(del_t))*MatrixLog6(pro);
vd = se3ToVec(vd_se3);

Xerr_se3 = MatrixLog6(X_inv*Xd);
Xerr = se3ToVec(Xerr_se3);

vb = adjoint*vd + Kp*Xerr + Ki*Xerr*del_t;
%vb
end

