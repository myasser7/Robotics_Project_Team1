function X = forward_kinematics_func(theta11, theta12, theta13)
    % This function computes the forward kinematics of a 3-joint robotic arm
    % Inputs:
    % theta11 - Joint angle for the first joint (in degrees)
    % theta12 - Joint angle for the second joint (in degrees)
    % theta13 - Joint angle for the third joint (in degrees)
    
    % Define robot parameters (lengths in cm)
    % l1 = 4.5;
    % l2 = 12.1;
    % l3 = 12.05;

    l1 = 4.045; % Length of link 1
    l2 = 12; % Length of link 2
    l3 = 21; % Length of link 3
    
    % Adjust theta2 and theta3 angles with an offset
    theta1 = theta11; 
    theta2 = theta12; % Adding 45 degrees to theta2
    theta3 = theta13; % Subtracting 45 degrees from theta3
    
    % Convert joint angles from degrees to radians for trigonometric calculations
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    
    % Compute the transformation matrix for each joint using DH parameters
    % T1 = transformation_func_Eq(theta1, l1, 1.374, pi/2); % First joint transformation
    % T2 = transformation_func_Eq(theta2, -1.952, l2, 0);     % Second joint transformation
    % T3 = transformation_func_Eq(theta3, -1.241, l3+3.654, 0);     % Third joint transformation

        % Define transformation matrices for each joint based on DH parameters
    T1 = transformation_func(theta1, l1, 0, pi/2); % First link
    T2 = transformation_func(theta2, 0, l2, 0);   % Second link
    T3 = transformation_func(theta3, 0, l3, 0);  % Third link
    
    % Multiply the individual transformation matrices to get the total transformation
    T_total = T1 * T2 * T3;
    
    % Extract the end-effector position from the total transformation matrix
    % The end-effector position is the 1st to 3rd rows of the 4th column of T_total
    X = T_total(1:3, 4); % X contains the [X, Y, Z] coordinates of the end-effector
end

% Helper function: transformation matrix based on DH parameters
function T = transformation_func_Eq(theta, d, a, alpha)
    % This function computes the transformation matrix for a given set of DH parameters
    % Inputs:
    % theta  - Joint angle (theta) (in radians)
    % d      - Link offset (d)
    % a      - Link length (a)
    % alpha  - Link twist (alpha)
    
    % Compute the 4x4 transformation matrix based on DH convention
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
