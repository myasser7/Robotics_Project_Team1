% Joint-Space Trajectory Tracking with Robot Visualization

clear; clc; close all;

% Time parameters
T = 10; % Duration in seconds
dt = 0.01; % Time step
t = 0:dt:T; % Time vector

% Initial and final positions for end-effector
x0 = 33; y0 = 0; z0 = 4.045;  % Initial position
xf = 0; yf = 0; zf = 37.045;   % Final position

% Inverse position kinematics to compute initial and final joint angles
[q1_0, q2_0, q3_0] = Inverse_Position_Kinematics(x0, y0, z0,0,2,2);
[q1_f, q2_f, q3_f] = Inverse_Position_Kinematics(xf, yf, zf,0,2,2);

% Joint velocities and accelerations are zero at start and end
q1_dot_0 = 0; q2_dot_0 = 0; q3_dot_0 = 0; % Initial velocities
q1_dot_f = 0; q2_dot_f = 0; q3_dot_f = 0; % Final velocities
q1_ddot_0 = 0; q2_ddot_0 = 0; q3_ddot_0 = 0; % Initial accelerations
q1_ddot_f = 0; q2_ddot_f = 0; q3_ddot_f = 0; % Final accelerations

% Solve for polynomial coefficients for each joint (5th degree polynomial)
coeffs_q1 = solve_5th_order_polynomial(q1_0, q1_f, q1_dot_0, q1_dot_f, q1_ddot_0, q1_ddot_f, T);
coeffs_q2 = solve_5th_order_polynomial(q2_0, q2_f, q2_dot_0, q2_dot_f, q2_ddot_0, q2_ddot_f, T);
coeffs_q3 = solve_5th_order_polynomial(q3_0, q3_f, q3_dot_0, q3_dot_f, q3_ddot_0, q3_ddot_f, T);

% Evaluate joint trajectories over time
q1 = evaluate_trajectory_5th_order(coeffs_q1, t);
q2 = evaluate_trajectory_5th_order(coeffs_q2, t);
q3 = evaluate_trajectory_5th_order(coeffs_q3, t);

% Robot arm link lengths
L1 = 4.045; % Length of link 1
L2 = 12;    % Length of link 2
L3 = 21;    % Length of link 3

% 3D Visualization of the Robot Trajectory
figure;
hold on;
grid on;
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('3DOF Robotic Arm Movement - Joint Space');
axis([-40 40 -40 40 0 40]);
view(3);

% Animate robot movement
for i = 1:10:length(t)
    % Convert joint angles to radians for computation
    q1_rad = deg2rad(q1(i));
    q2_rad = deg2rad(q2(i));
    q3_rad = deg2rad(q3(i));

    % Joint positions
    x1 = 0; y1 = 0; z1 = L1; % Base to joint 1
    x2 = L2 * cos(q2_rad) * cos(q1_rad);
    y2 = L2 * cos(q2_rad) * sin(q1_rad);
    z2 = L1 + L2 * sin(q2_rad);
    x3 = x2 + L3 * cos(q2_rad + q3_rad) * cos(q1_rad);
    y3 = y2 + L3 * cos(q2_rad + q3_rad) * sin(q1_rad);
    z3 = z2 + L3 * sin(q2_rad + q3_rad);

    % Clear previous frame
    cla;

    % Plot robot links
    plot3([0, x2, x3], [0, y2, y3], [z1, z2, z3], 'k-o', 'LineWidth', 2, ...
          'MarkerFaceColor', 'r', 'MarkerSize', 6);
    hold on;

    % Highlight current end-effector position
    plot3(x3, y3, z3, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

    % Maintain plot settings
    axis([-40 40 -40 40 0 40]);
    view(3);

    % Pause for animation
    pause(0.01);
end

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
    q_ddot = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end

disp('Joint trajectories calculated successfully.');
% Display polynomial functions for each joint
fprintf('Polynomial function for q1(t): %.4f + %.4f*t + %.4f*t^2 + %.4f*t^3 + %.4f*t^4 + %.4f*t^5\n', coeffs_q1);
fprintf('Polynomial function for q2(t): %.4f + %.4f*t + %.4f*t^2 + %.4f*t^3 + %.4f*t^4 + %.4f*t^5\n', coeffs_q2);
fprintf('Polynomial function for q3(t): %.4f + %.4f*t + %.4f*t^2 + %.4f*t^3 + %.4f*t^4 + %.4f*t^5\n', coeffs_q3);

q_matrix = [q1 ; q2 ; q3];
q1_series = timeseries(q1 , t); %first column
q2_series = timeseries(q2 , t); %second column
q3_series = timeseries(q3 , t); %third column
disp('joint pos. at each time step');
disp(q_matrix);