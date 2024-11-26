% Joint-Space Trajectory Tracking Using Third-Degree Polynomials
clear; clc;

% Time parameters
T = 10; % Duration in seconds
t = linspace(0, T, 1000); % Time vector

% Initial and final positions for end-effector
x0 = 5; y0 = 5; z0 = 5; % Initial positions
xf = 5; yf = -5; zf = 5; % Final positions

% Inverse position kinematics to compute initial and final joint angles
[q1_0, q2_0, q3_0] = Inverse_Position_Kinematics(x0, y0, z0);
[q1_f, q2_f, q3_f] = Inverse_Position_Kinematics(xf, yf, zf);

% Joint velocities and accelerations are zero at start and end
q1_dot_0 = 0; q2_dot_0 = 0; q3_dot_0 = 0; % Initial velocities
q1_dot_f = 0; q2_dot_f = 0; q3_dot_f = 0; % Final velocities
q1_ddot_0 = 0; q2_ddot_0 = 0; q3_ddot_0 = 0; % Initial accelerations
q1_ddot_f = 0; q2_ddot_f = 0; q3_ddot_f = 0; % Final accelerations

% Solve for polynomial coefficients for each joint (3rd degree polynomial)
% q(t) = a0 + a1*t + a2*t^2 + a3*t^3
coeffs_q1 = solve_polynomial(q1_0, q1_f, q1_dot_0, q1_dot_f, T);
coeffs_q2 = solve_polynomial(q2_0, q2_f, q2_dot_0, q2_dot_f, T);
coeffs_q3 = solve_polynomial(q3_0, q3_f, q3_dot_0, q3_dot_f, T);

% Evaluate joint trajectories over time
[q1, q1_dot, q1_ddot] = evaluate_trajectory(coeffs_q1, t);
[q2, q2_dot, q2_ddot] = evaluate_trajectory(coeffs_q2, t);
[q3, q3_dot, q3_ddot] = evaluate_trajectory(coeffs_q3, t);

% Plot joint trajectories
figure;
subplot(3,1,1);
plot(t, q1, 'r', t, q2, 'g', t, q3, 'b');
legend('q1(t)', 'q2(t)', 'q3(t)');
title('Joint Positions');
xlabel('Time (s)'); ylabel('Angle (degrees)');

subplot(3,1,2);
plot(t, q1_dot, 'r', t, q2_dot, 'g', t, q3_dot, 'b');
legend('q1_dot(t)', 'q2_dot(t)', 'q3_dot(t)');
title('Joint Velocities');
xlabel('Time (s)'); ylabel('Velocity (deg/s)');

subplot(3,1,3);
plot(t, q1_ddot, 'r', t, q2_ddot, 'g', t, q3_ddot, 'b');
legend('q1_ddot(t)', 'q2_ddot(t)', 'q3_ddot(t)');
title('Joint Accelerations');
xlabel('Time (s)'); ylabel('Acceleration (deg/s^2)');


% Helper function: Solve polynomial coefficients
function coeffs = solve_polynomial(q0, qf, q_dot_0, q_dot_f, T)
    % Third-degree polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    % Boundary conditions
    A = [1, 0, 0, 0;        % q(0) = q0
         0, 1, 0, 0;        % q_dot(0) = q_dot_0
         1, T, T^2, T^3;    % q(T) = qf
         0, 1, 2*T, 3*T^2]; % q_dot(T) = q_dot_f
    b = [q0; q_dot_0; qf; q_dot_f];
    coeffs = A\b; % Solve linear system for coefficients [a0; a1; a2; a3]
end

% Helper function: Evaluate trajectory
function [q, q_dot, q_ddot] = evaluate_trajectory(coeffs, t)
    % Polynomial coefficients
    a0 = coeffs(1); a1 = coeffs(2); a2 = coeffs(3); a3 = coeffs(4);
    
    % Evaluate position, velocity, and acceleration
    q = a0 + a1*t + a2*t.^2 + a3*t.^3;
    q_dot = a1 + 2*a2*t + 3*a3*t.^2;
    q_ddot = 2*a2 + 6*a3*t;
end
