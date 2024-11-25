clc;

%Forward & Inverse Position Kinematics


%Forward Position Kinematics X Y Z
X = forward_kinematics_func_Eq([deg2rad(10), deg2rad(20), deg2rad(30)])

% Define the desired end-effector position
Xe = X(1);  % X-coordinate
Ye = X(2);   % Y-coordinate
Ze = X(3);  % Z-coordinate

%Inverse Position Kinematics

% Solve for joint angles
[q1, q2, q3] = Inverse_Position_Kinematics(Xe, Ye, Ze);

% Display results
fprintf('Joint angles:\nq1 = %.2f°\nq2 = %.2f°\nq3 = %.2f°\n', q1, q2, q3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Forward & Inverse Velocity Kinematics

%Forward Velocity Kinematics
% Input joint angles (degrees) and velocities (deg/s)
q1 = 30; q2 = 45; q3 = 60;        % Joint angles
q1_dot = 10; q2_dot = 5; q3_dot = 15; % Joint velocities

% Call the function
[V_sym, V, J_sym, J_num] = forward_Velocity_func(q1, q2, q3, q1_dot, q2_dot, q3_dot);

% Display results
disp('Symbolic End-Effector Velocity (V_sym):');
disp(V_sym);

disp('Numeric End-Effector Velocity (V):');
disp(V);

disp('Symbolic Jacobian (J_sym):');
disp(J_sym);

disp('Numeric Jacobian (J_num):');
disp(J_num);

% Call the inverse velocity function
q_dot = inverse_velocity_func(V, q1, q2, q3);

% Display the joint velocities
disp('Joint Velocities (q_dot):');
disp(q_dot);
