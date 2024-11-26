% Task-Space Trajectory Planning for 3DOF Robot Using Inverse Kinematics

clc;
clear;
close all;

% Define initial and final positions in Cartesian space
x0 = 33; y0 = 0; z0 = 4.045;  % Initial position
xf = 9.3630; yf = 7.8565; zf = 30.9651;   % Final position

% Define total time and time step
T = 10;          % Total duration in seconds
dt = 0.1;       % Time step
t = 0:dt:T;      % Time vector

% Interpolate x, y, z positions linearly over time
x_traj = linspace(x0, xf, length(t));
y_traj = linspace(y0, yf, length(t));
z_traj = linspace(z0, zf, length(t));

% Initialize joint angles for storage
q1_traj = zeros(1, length(t));
q2_traj = zeros(1, length(t));
q3_traj = zeros(1, length(t));

q1_traj_rad = zeros(1, length(t));
q2_traj_rad = zeros(1, length(t));
q3_traj_rad = zeros(1, length(t));

% Initial guess for joint angles (example: zero configuration)
q1_guess = 1; q2_guess = 100; q3_guess = 1;

% Robot link lengths (example values, adjust based on your robot)
L1 = 4.045; % Length of link 1
L2 = 12;    % Length of link 2
L3 = 21;    % Length of link 3

% Calculate joint angles using inverse kinematics for each time step
for i = 1:length(t)
    % Current end-effector position
    x = x_traj(i);
    y = y_traj(i);
    z = z_traj(i);

    % Use Inverse Kinematics function with current position and previous guess
    % try
    %     [q1, q2, q3] = Inverse_Position_Kinematics(x, y, z, q1_guess, q2_guess, q3_guess);
    % catch exception
    %     error('Inverse kinematics failed at step %d: %s', i, exception.message);
    % end
    [q1, q2, q3] = Inverse_Position_Kinematics(x, y, z, q1_guess, q2_guess, q3_guess);

    

    % Store joint angles
    q1_traj(i) = q1;
    q2_traj(i) = q2;
    q3_traj(i) = q3;

    q1_traj_rad(i) = deg2rad(q1);
    q2_traj_rad(i) = deg2rad(q2);
    q3_traj_rad(i) = deg2rad(q3);


    % Update guesses for the next iteration
    q1_guess = q1;
    q2_guess = q2;
    q3_guess = q3;
end

% Visualization: Single 3D View of the Robot and Trajectory
figure;
hold on;

% Set up the 3D plot
grid on;
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('3DOF Robotic Arm Movement and Trajectory');
axis([-40 40 -40 40 0 40]); % Adjust axis limits as needed
view(3); % Set a 3D perspective

% Plot the desired trajectory in advance
plot3(x_traj, y_traj, z_traj, 'b--', 'LineWidth', 1.5); % Planned trajectory
plot3(x0, y0, z0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start point
plot3(xf, yf, zf, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End point

% Animate the robot and its movement
for i = 1:10:length(t)
    % Joint positions
    x1 = 0; y1 = 0; z1 = L1;          % Base to joint 1
    x2 = L2 * cos(q2_traj_rad(i)) * cos(q1_traj_rad(i));
    y2 = L2 * cos(q2_traj_rad(i)) * sin(q1_traj_rad(i));
    z2 = L1 + L2 * sin(q2_traj_rad(i));
    x3 = x2 + L3 * cos(q2_traj_rad(i) + q3_traj_rad(i)) * cos(q1_traj_rad(i));
    y3 = y2 + L3 * cos(q2_traj_rad(i) + q3_traj_rad(i)) * sin(q1_traj_rad(i));
    z3 = z2 + L3 * sin(q2_traj_rad(i) + q3_traj_rad(i));

    % Clear the previous frame
    cla;

    % Plot the robot's configuration
    plot3([0, x2, x3], [0, y2, y3], [z1, z2, z3], 'k-o', 'LineWidth', 2, ...
        'MarkerFaceColor', 'r', 'MarkerSize', 6);
    hold on;

    % Plot the trajectory traced so far
    plot3(x_traj(1:i), y_traj(1:i), z_traj(1:i), 'b', 'LineWidth', 1.5);

    % Highlight the current end-effector position
    plot3(x_traj(i), y_traj(i), z_traj(i), 'go', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'g');

    % Plot the start and final points
    plot3(x0, y0, z0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start point
    plot3(xf, yf, zf, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End point

    % Maintain axis properties
    axis([-40 40 -40 40 0 40]); % Adjust axis limits as needed
    view(3); % Set a 3D perspective

    % Pause for animation
    pause(0.01);
end

Matrix_q = [q1_traj;q2_traj;q3_traj];
q1_series = timeseries(q1_traj,t);
q2_series = timeseries(q2_traj,t);
q3_series = timeseries(q3_traj,t);

disp(Matrix_q);