%first trajectory 
function [q1_1 , q2_1 , q3_1 , t1] = first_traj()

T1 = 5; % Duration in seconds
dt = 0.1; % Time step
t1 = 0:dt:T1; % Time vector

% Initial and final positions for end-effector
x0 = 0; y0 = 29.4853; z0 = 12.5;  % Initial position
xf = 0; yf = 29.4853; zf = 0;   % Final position

% Inverse position kinematics to compute initial and final joint angles
[q1_0, q2_0, q3_0] = Inverse_Position_Kinematics(x0, y0, z0,90,30,-30);
[q1_f, q2_f, q3_f] = Inverse_Position_Kinematics(xf, yf, zf,90,30,-30);

% Joint velocities and accelerations are zero at start and end
q1_dot_0 = 0; q2_dot_0 = 0; q3_dot_0 = 0; % Initial velocities
q1_dot_f = 0; q2_dot_f = 0; q3_dot_f = 0; % Final velocities
q1_ddot_0 = 0; q2_ddot_0 = 0; q3_ddot_0 = 0; % Initial accelerations
q1_ddot_f = 0; q2_ddot_f = 0; q3_ddot_f = 0; % Final accelerations

% Solve for polynomial coefficients for each joint (5th degree polynomial)
coeffs_q1_1 = solve_5th_order_polynomial(q1_0, q1_f, q1_dot_0, q1_dot_f, q1_ddot_0, q1_ddot_f, T1);
coeffs_q2_1 = solve_5th_order_polynomial(q2_0, q2_f, q2_dot_0, q2_dot_f, q2_ddot_0, q2_ddot_f, T1);
coeffs_q3_1 = solve_5th_order_polynomial(q3_0, q3_f, q3_dot_0, q3_dot_f, q3_ddot_0, q3_ddot_f, T1);

% Evaluate joint trajectories over time
q1_1 = evaluate_trajectory_5th_order(coeffs_q1_1, t1);
q2_1 = evaluate_trajectory_5th_order(coeffs_q2_1, t1);
q3_1 = evaluate_trajectory_5th_order(coeffs_q3_1, t1);
% q1_final = q1_1
% q2_final = q2_1
% q3_final = q3_1
% 
% q1_series = timeseries(q1_final , t1); %first column
% q2_series = timeseries(q2_final , t1); %second column
% q3_series = timeseries(q3_final , t1); %third column
% Helper function: Solve 5th order polynomial coefficients
function coeffs = solve_5th_order_polynomial(q0, qf, q_dot_0, q_dot_f, q_ddot_0, q_ddot_f, T)
    % Fifth-degree polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    % Boundary conditions
    A = [1, 0, 0, 0, 0, 0;            % q(0) = q0
         0, 1, 0, 0, 0, 0;            % q_dot(0) = q_dot_0
         0, 0, 2, 0, 0, 0;            % q_ddot(0) = q_ddot_0
         1, T, T^2, T^3, T^4, T^5;    % q(T) = qf
         0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4; % q_dot(T) = q_dot_f
         0, 0, 2, 6*T, 12*T^2, 20*T^3]; % q_ddot(T) = q_ddot_f
    b = [q0; q_dot_0; q_ddot_0; qf; q_dot_f; q_ddot_f];
    coeffs = A\b; % Solve linear system for coefficients [a0; a1; a2; a3; a4; a5]
end

% Helper function: Evaluate trajectory for 5th order polynomial
function [q, q_dot, q_ddot] = evaluate_trajectory_5th_order(coeffs, t)
    % Polynomial coefficients
    a0 = coeffs(1); a1 = coeffs(2); a2 = coeffs(3); 
    a3 = coeffs(4); a4 = coeffs(5); a5 = coeffs(6);
    
    % Evaluate position, velocity, and acceleration
    q = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    q_dot = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    q_ddot = 2*a2 + 6*a3*t + 12*a4*t.^2 +20*a5*t.^3;
end
end