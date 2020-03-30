function [traj] = reference_trajectory(Tse_initial,Tsc_initial,Tsc_final,Tce_standoff,Tce_grasp)
% Venkatesh Prasad Venkataramanan
% PID : A53318036

% Constants %
k = 1;
gripper_state_open = 0;
gripper_state_closed = 1;
traj3 = [];
traj7 = [];
%%%%%%%%%%%%%%

% Calculating some transformation matrices %
Tse_standoff = Tsc_initial*Tce_standoff;
Tse_grasp = Tsc_initial*Tce_grasp;
Tse_standoff_final = Tsc_final*Tce_standoff;
Tse_grasp_final = Tsc_final*Tce_grasp;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tse_initial to Tce_standoff(i.e Tse_standoff) (1) %%%
traj1 = ScrewTrajectory(Tse_initial, Tse_standoff, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tce_standoff to Tce_grasp (2)%%%

traj2 = ScrewTrajectory(Tse_standoff, Tse_grasp, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%

%%% Closing in (3) %%%%%%%%%%%%%%%%%%%
close3 = traj2(1,400);
for i=1:500
    traj3 = [traj3 close3];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tce_grasp to Tce_standoff (4)%%%
traj4 = ScrewTrajectory(Tse_grasp, Tse_standoff, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tce_standoff to Tce_standoff(final) (5)%%%
traj5 = ScrewTrajectory(Tse_standoff, Tse_standoff_final, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tce_standoff(final) to Tce_grasp(final) (6)%%%
traj6 = ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Letting go(7) %%%
open7 = traj6(1,400);
for i=1:500
    traj7 = [traj7 open7];
end
%%%%%%%%%%%%%%%%%%%

%%% Tce_grasp(final) to Tce_standoff(final) (8)%%%
traj8 = ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Tce_standoff(final) to Tse_initial (9)%%%
traj9 = ScrewTrajectory(Tse_standoff_final, Tse_initial, k, 400, 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Putting together into a big matrix %%%
traj = [traj1 traj2 traj3 traj4 traj5 traj6 traj7 traj8]; %traj9];
%traj = [traj1; traj2; traj3; traj4; traj5; traj6; traj7; traj8; traj9];
end

