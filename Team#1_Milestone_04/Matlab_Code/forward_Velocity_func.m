function [V_sym, V, J_sym, J_num] = forward_Velocity_func(q1, q2, q3, q1_dot, q2_dot, q3_dot)
    % Declare symbolic variables for joint angles and their derivatives
    syms theta1 theta2 theta3
    syms theta1_dot theta2_dot theta3_dot

    % Link lengths
    l1 = 4.045; % Length of link 1
    l2 = 12;    % Length of link 2
    l3 = 21;    % Length of link 3

    % Define transformation matrices for each joint based on DH parameters
    T1 = transformation_func(theta1, l1, 0, pi/2); % First link
    T2 = transformation_func(theta2, 0, l2, 0);    % Second link
    T3 = transformation_func(theta3, 0, l3, 0);    % Third link

    % Total transformation from base to end-effector
    T_total = T1 * T2 * T3;

    % Define forward kinematics to compute end-effector position symbolically
    X = T_total(1:3, 4); % Extract the position of the end-effector
    fx = [X(1); X(2); X(3)]; % End-effector position vector

    % Compute the positional Jacobian (Jv) symbolically
    Jv = jacobian(fx, [theta1; theta2; theta3]);

    % Define rotational Jacobians (Jw) using rotational axes of transformations
    Jw1 = [0; 0; 1];               % Base rotation axis (Z-axis of base)
    Jw2 = T1(1:3, 3);              % Z-axis of T1
    T1_T2 = T1 * T2;               % Temporary variable for T1 * T2
    Jw3 = T1_T2(1:3, 3);           % Z-axis of T1 * T2 (for joint 3)

    % Combine all rotational Jacobians
    Jw = [Jw1, Jw2, Jw3];

    % Assemble the total Jacobian matrix (6x3) symbolically
    J_sym = [Jv; Jw];

    % Define symbolic vector of joint velocities
    q_dot = [theta1_dot; theta2_dot; theta3_dot];

    % Compute the symbolic end-effector velocity
    V_sym = vpa(J_sym * q_dot); % Symbolic velocity

    % Evaluate the numeric Jacobian matrix (J_num) at the specified joint angles
    J_num = double(subs(J_sym, [theta1, theta2, theta3], [q1, q2, q3]));

    % Compute the numeric end-effector velocity (V) using given joint velocities
    V = J_num * [q1_dot; q2_dot; q3_dot];

end

% Helper function for DH transformation matrix computation
function T = transformation_func(theta, d, a, alpha)
    % Define the transformation matrix based on the Denavit-Hartenberg parameters
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
