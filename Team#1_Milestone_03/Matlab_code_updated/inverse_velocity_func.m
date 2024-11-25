function q_dot = inverse_velocity_func(V_desired, q1, q2, q3)
    % Symbolic variables for joint angles
    syms theta1 theta2 theta3

    % Link lengths
    l1 = 4.045; % Length of link 1
    l2 = 12;    % Length of link 2
    l3 = 21;    % Length of link 3

    % Define transformation matrices using DH parameters
    T1 = transformation_func(theta1, l1, 0, pi/2); % First link
    T2 = transformation_func(theta2, 0, l2, 0);    % Second link
    T3 = transformation_func(theta3, 0, l3, 0);    % Third link

    % Total transformation from base to end-effector
    T1_T2 = T1 * T2;       % Transformation to second joint
    T_total = T1_T2 * T3;  % Transformation to end-effector

    % End-effector position vector from forward kinematics
    fx = T_total(1:3, 4); % Extract position (x, y, z)

    % Compute positional Jacobian (Jv)
    q = [theta1; theta2; theta3];    % Joint variables
    Jv = jacobian(fx, q);           % Positional Jacobian

    % Rotational Jacobians for each joint
    Jw1 = [0; 0; 1];                % Z-axis of the base
    Jw2 = T1(1:3, 3);               % Z-axis after first transformation
    Jw3 = T1_T2(1:3, 3);            % Z-axis after second transformation

    % Combine rotational Jacobians
    Jw = [Jw1, Jw2, Jw3];

    % Assemble full Jacobian (6x3)
    J = [Jv; Jw];

    % Substitute joint angles for numerical evaluation
    J_num = double(subs(J, [theta1, theta2, theta3], [q1, q2, q3]));

    % Compute pseudo-inverse of the Jacobian using MATLAB's pinv()
    Pseudo_J = pinv(J_num);

    % Compute joint velocities (q_dot)
    q_dot = Pseudo_J * V_desired;
end

% Helper function to compute the transformation matrix using DH parameters
function T = transformation_func(theta, d, a, alpha)
    % Define the transformation matrix for given DH parameters
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
