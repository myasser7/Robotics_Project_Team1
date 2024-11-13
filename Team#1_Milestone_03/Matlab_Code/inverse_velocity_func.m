function q_dot = inverse_velocity_func(V_desired, q1, q2, q3)
    % Calculate the Jacobian matrix symbolically
    syms theta1 theta2 theta3
    l1 = 4.5; % Length of link 1
    l2 = 12.1; % Length of link 2
    l3 = 12.05; % Length of link 3

     % Define transformation matrices for each joint based on DH parameters
    T1 = transformation_func(theta1, l1, 1.374, pi/2); % First link
    T2 = transformation_func(theta2, -1.952, l2, 0);   % Second link
    T3 = transformation_func(theta3, -1.241, l3+3.654, 0);  % Third link
    
    % Joint angle vector
    q = [theta1; theta2; theta3];
    
    % Forward kinematics to get the end-effector position
    X = forward_kinematics_func_Eq();
    f1 = X(1);
    f2 = X(2);
    f3 = X(3);
    fx = [f1; f2; f3]; % Position vector of the end-effector
    
    % Compute the positional Jacobian (Jv) by differentiating position with respect to joint angles
    Jv = jacobian(fx, q);
    
    % Rotation components of the Jacobian for each joint axis
    Jw1 = [0; 0; 1];
    Jw2 = T1(1:3, 3);
    Jw3 = T2(1:3, 3);
    Jw = [Jw1, Jw2, Jw3]; % Combine rotational components into Jw
    
    % Complete Jacobian by combining Jv and Jw
    J = [Jv; Jw];
    
    % Substitute actual joint angles into the Jacobian matrix for numerical evaluation
    J_num = double(subs(J, [theta1, theta2, theta3], [q1, q2, q3]));
    
    Pseudo_J = inv(transpose(J_num)*J_num)*transpose(J_num);
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
