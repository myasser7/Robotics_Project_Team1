function [V_sym, V, J_sym ,J_num] = forward_Velocity_func(q1, q2, q3, q1_dot, q2_dot, q3_dot)
    % Declare symbolic variables for joint angles and their derivatives
    syms theta1 theta2 theta3
    syms theta1_dot theta2_dot theta3_dot

    l1 = 4.5; % Length of link 1
    l2 = 12.1; % Length of link 2
    l3 = 12.05; % Length of link 3

    % Define transformation matrices for each joint based on DH parameters
    T1 = transformation_func(theta1, l1, 1.374, pi/2); % First link
    T2 = transformation_func(theta2, -1.952, l2, 0);   % Second link
    T3 = transformation_func(theta3, -1.241, l3+3.654, 0);  % Third link

    % Define rotational axes for each joint
    Jw1 = [0; 0; 1];           % Base rotation axis for the first joint
    Jw2 = T1(1:3, 3);          % Z-axis of first transformation for the second joint
    Jw3 = T2(1:3, 3);          % Z-axis of second transformation for the third joint

    % Set up symbolic vectors for angles and their derivatives
    q = [theta1; theta2; theta3];
    q_dot = [theta1_dot; theta2_dot; theta3_dot];

    % Compute the forward kinematics to get end-effector position
    X = forward_kinematics_func_Eq();
    f1 = X(1); % x-coordinate of end-effector
    f2 = X(2); % y-coordinate of end-effector
    f3 = X(3); % z-coordinate of end-effector
    fx = [f1; f2; f3]; % End-effector position vector

    % Compute the symbolic positional Jacobian (Jv)
    jv = jacobian(fx, q);

    % Combine rotational Jacobians (Jw) into a single symbolic matrix
    w = [Jw1, Jw2, Jw3]; % Rotational component Jacobians

    % Construct the total Jacobian matrix (J_sym) symbolically
    J_sym = [jv; w];

    % Compute the symbolic end-effector velocity (V_sym) by multiplying J_sym with q_dot
    V_sym = vpa(J_sym * q_dot);

    % Evaluate the numeric Jacobian matrix (J_num) at the specified joint angles
    J_num = double(subs(J_sym, [theta1, theta2, theta3], [q1, q2, q3]));

    % Calculate the numeric end-effector velocity (V) using evaluated joint velocities
    V = double(subs(V_sym, [theta1, theta2, theta3, theta1_dot, theta2_dot, theta3_dot], ...
                    [q1, q2, q3, q1_dot, q2_dot, q3_dot]));

end

% Helper function for DH transformation matrix computation
function T = transformation_func(theta, d, a, alpha)
    % Define the transformation matrix based on the Denavit-Hartenberg parameters
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
