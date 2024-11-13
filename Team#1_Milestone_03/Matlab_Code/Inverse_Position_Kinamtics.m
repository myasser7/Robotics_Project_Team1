function [q1, q2, q3] = inverse_position_kinematics(Xe, Ye, Ze)
    % Set up symbolic variables for joint angles
    syms theta1 theta2 theta3
    
    % Parameters for the Newton-Raphson method
    max_iterations = 500;      % Maximum number of iterations
    tolerance = 1e-3;          % Convergence tolerance for error
    
    % Initial guess for joint angles (in degrees)
    initial_guess = [20; 10; 30];
    current_guess = initial_guess;
    
    % Define forward kinematics using symbolic function
    X = forward_kinematics_func_Eq();
    
    % Desired end-effector positions
    f1 = X(1) - Xe;
    f2 = X(2) - Ye;
    f3 = X(3) - Ze;
    f_vector = [f1; f2; f3];       % Error function
    
    % Jacobian matrix with respect to joint angles
    J = jacobian(f_vector, [theta1; theta2; theta3]);
    
    % Iterate until convergence or maximum iterations reached
    for i = 1:max_iterations
        % Current joint angle values in radians
        theta1_val = deg2rad(current_guess(1));
        theta2_val = deg2rad(current_guess(2));
        theta3_val = deg2rad(current_guess(3));
        
        % Substitute current joint values into error function and Jacobian
        f_val = double(subs(f_vector, {theta1, theta2, theta3}, {theta1_val, theta2_val, theta3_val}));
        J_val = double(subs(J, {theta1, theta2, theta3}, {theta1_val, theta2_val, theta3_val}));
        
        % Update joint angles using Newton-Raphson method
        delta_q = -J_val \ f_val;
        new_guess = current_guess + rad2deg(delta_q);
        
        % Compute the error and check for convergence
        error = norm(new_guess - current_guess);
        if error < tolerance
            fprintf('Converged in %d iterations\n', i);
            break;
        end
        
        % Update the current guess
        current_guess = new_guess;
    end
    
    % Return final joint angles
    q1 = current_guess(1);
    q2 = current_guess(2);
    q3 = current_guess(3);
    
    if i == max_iterations
        warning('Maximum iterations reached without convergence.');
    end
end

% Helper function for forward kinematics
function X = forward_kinematics_func_Eq()
    % Define symbolic joint angles and link lengths
    syms theta1 theta2 theta3
    l1 = 4.5; l2 = 12.1; l3 = 12.05;
    
    % Transformation matrices using DH parameters
    T1 = transformation_func(theta1, l1, 1.374, pi/2);
    T2 = transformation_func(theta2, -1.952, l2, 0);
    T3 = transformation_func(theta3, -1.241, l3+3.654, 0);
    
    % Total transformation from base to end-effector
    T_total = T1 * T2 * T3;
    X = T_total(1:3, 4); % Extract the position of the end-effector
end

% Transformation matrix function for DH parameters
function T = transformation_func(theta, d, a, alpha)
    % Define the transformation matrix for given DH parameters
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
