% Venkatesh Prasad Venkataramanan
% PID : A53318036

% Values to experiment with. These are the best case parameters %
Kp = 10 * eye(6);
Ki = 7 * eye(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Some declarations(current state, control input, etc %
del_t = 0.01;
curr = [0.323,-0.5,-0.5, 0, 0, 0, 0, 0, -pi/4, pi/4, -pi/4, pi/4];
cont = zeros(1,9);
gripper_state_open = 0;
gripper_state_closed = 1;
% Gotta initialise this properly %
%csv_matrix_sans_gs = curr;
%%%%%%%%%%%%%%%%%%%%%%%%%

% Blist %
Blist = [[0; 0; 1;   0; 0.033; 0], ...
       [0; -1; 0;   -0.5076;   0;   0], ...
       [0; -1; 0;   -0.3526;   0;   0], ...
       [0; -1; 0; -0.2176; 0; 0], ...
       [0; 0; 1; 0; 0; 0]];
%%%%%%%%%%%%%%

% A lot of initialisations %
q_ci = [0, 1, 0];
q_cg = [-pi/2, 0, -1];
q_bi = [0.323,-0.5,-0.5];
M0e = [[1, 0, 0, 0.033]; [0, 1, 0, 0]; [0, 0, 1, 0.6546]; [0, 0, 0, 1]];
Tsb_initial = [[cos(q_bi(1)), -sin(q_bi(1)), 0, q_bi(2)]; 
             [sin(q_bi(1)), cos(q_bi(1)), 0, q_bi(3)]; 
             [0, 0, 1, 0.0963]; 
             [0, 0, 0, 1]];
Tb0 = [[1, 0, 0, 0.1662]; [0, 1, 0, 0]; [0, 0, 1, 0.0026]; [0, 0, 0, 1]];
Ts0 = Tsb_initial*Tb0;
thetalist = [0;0;0;0;0];
T0e = FKinBody(M0e,Blist,thetalist);
%Tse_initial = Ts0*M;
%Tse_initial = Ts0*T0e;
Tse_initial = [0 0 1 0;0 1 0 0;-1 0 0 0.5;0 0 0 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Declare other inputs %
Tsc_initial = [[cos(q_ci(1)), -sin(q_ci(1)), 0, q_ci(2)]; 
             [sin(q_ci(1)), cos(q_ci(1)), 0, q_ci(3)]; 
             [0, 0, 1, 0.025]; 
             [0, 0, 0, 1]];
Tsc_final = [[cos(q_cg(1)), -sin(q_cg(1)), 0, q_cg(2)]; 
             [sin(q_cg(1)), cos(q_cg(1)), 0, q_cg(3)]; 
             [0, 0, 1, 0.025]; 
             [0, 0, 0, 1]];
% Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
% Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];



Tce_grasp = [cos(3*pi/4) 0 sin(3*pi/4) 0; 0 1 0 0; -sin(3*pi/4) 0 cos(3*pi/4) 0; 0 0 0 1];
Tce_standoff = [cos(3*pi/4) 0 sin(3*pi/4) 0; 0 1 0 0; -sin(3*pi/4) 0 cos(3*pi/4) 0.1; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculating initial Tse_desired and next Tse_desired %

traj = reference_trajectory(Tse_initial,Tsc_initial,Tsc_final,Tce_standoff,Tce_grasp);
%%%%%%%%%%%%%%%%%%%%%%%%
Tse_desired = traj{1};
Tse_desired_next = traj{2};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specifying initial Tse_actual %
%Tse_actual = Tse_desired;
Tse_actual = Ts0*T0e;
% Initialising csv_matrix %
csv_matrix_sans_gs = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Some pre-calculations for calcJac() and NextState() %
% Calculate F-Matrix for calcJac%
r = 0.0475;
w = 0.15;
l = 0.235;
F = [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0];
mul = (r/4)*F;
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate F-Matrix for Next_State%
F_next = [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
mul_next = (r/4)*F_next;
%%%%%%%%%%%%%%%%%%%%%%%%%%

Xerr_save = [];
% Main test loop %
for i=1:3198
    [vb,Xerr] = feedbackcontrol(Tse_actual,Tse_desired,Tse_desired_next,Kp,Ki,del_t);
    Je_pinv = calcJac(curr(4:8)',M0e,Tb0,mul);
    product = Je_pinv * vb;
    cont(6:9) = product(1:4);
    cont(1:5) = product(5:9);
    % Calculate Tse_actual for next loop %
    nextstate = Next_State(curr,cont,mul_next,del_t);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % Setting current to nextstate %
    curr = nextstate;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % CSV Matrix  Sans Gripper State and Error Matrix Accumulation %
    csv_matrix_sans_gs = [csv_matrix_sans_gs; curr];
    Xerr_save = [Xerr_save Xerr];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate Tse_actual for next loop %
    Tsb_q = [[cos(curr(1)), -sin(curr(1)), 0, curr(2)]; [sin(curr(1)), cos(curr(1)), 0, curr(3)]; [0, 0, 1, 0.0963]; [0, 0, 0, 1]];
    Ts0 = Tsb_q*Tb0;
    T0e = FKinBody(M0e, Blist, curr(4:8)');
    Tse_actual = Ts0*T0e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate Tse_desired and Tse_desired_next for next loop %  
    Tse_desired = traj{i+1};    
    Tse_desired_next = traj{i+2};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
end

%%%%%%%%%%%%
f = fopen('output_best.csv', 'w');
%%%%%%%%%%%%%%%%%%%

%Tse_actual_matrix;
disp('Generating CSV file for simulation');
% Loop to write final CSV file %
for i=1:399    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_open);    
end

for i=400:799    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_open);    
end

for i=800:1199    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_closed);    
end

for i=1200:1599    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_closed);    
end

for i=1600:1999    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_closed);    
end

for i=2000:2399    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_closed);    
end

for i=2400:2799    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_open);    
end

for i=2800:3198    
    fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_open);    
end

% for i=3200:3598    
%     fprintf(f, ' %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d\n', csv_matrix_sans_gs(i,:), gripper_state_open);    
% end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fclose(f);
N = 3199;
disp('Generating Error Plot');
plot(1:N-1, Xerr_save(1,:))
hold on
plot(1:N-1, Xerr_save(2,:))
hold on
plot(1:N-1, Xerr_save(3,:))
hold on
plot(1:N-1, Xerr_save(4,:))
hold on
plot(1:N-1, Xerr_save(5,:))
hold on
plot(1:N-1, Xerr_save(6,:))
hold off
title('Error Evolution over Time')

disp('Saving to file...');
ferror = fopen('error_best.csv', 'w');
for i=1:N-1
        fprintf(ferror, '%10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n', Xerr_save(:,i));    
end 
fclose(ferror);
disp('Done');