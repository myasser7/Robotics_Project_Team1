function [q1, q2, q3] = Inverse_Position_Kinematics(Xe, Ye, Ze,Xi, Yi, Zi)
    % Set up symbolic variables for joint angles
    syms theta1 theta2 theta3
    
    % Parameters for the Newton-Raphson method
    max_iterations = 500;      % Maximum number of iterations
    tolerance = 1e-6;          % Convergence tolerance for error
    
    % Initial guess for joint angles (in degrees)
    initial_guess = [Xi; Yi; Zi]; % You can modify this as needed
    current_guess = initial_guess;
    
    % Define forward kinematics symbolically
    X_symbolic = forward_kinematics_func_Eq();
    
    % Desired end-effector positions
    f1 = X_symbolic(1) - Xe;
    f2 = X_symbolic(2) - Ye;
    f3 = X_symbolic(3) - Ze;
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
        if rank(J_val) < 3
            error('Jacobian is singular; inverse kinematics cannot be solved.');
        end
        
        delta_q = -J_val \ f_val;
        new_guess = current_guess + rad2deg(delta_q);
        
        % Ensure joint angles remain between 0° and 360° using mod
        new_guess = mod(new_guess, 360);
        
        % Compute the error and check for convergence
        error = norm(f_val);
        if error < tolerance
            fprintf('Converged in %d iterations\n', i);
            break;
        end
        
        % Update the current guess
        current_guess = new_guess;
    end
    
    % Check if the solution converged
    if i == max_iterations
        warning('Maximum iterations reached without convergence.');
    end
    
    % Return final joint angles
    % if(current_guess(1)>=180)
    %     q1 = current_guess(1)-180;
    % else
    %     q1 =  current_guess(1);
    % end
    % 
    % if(current_guess(2)>=180)
    %     q2 = current_guess(2)-180;
    % else
    %     q2 =  current_guess(2);
    % end
    % 
    % if(current_guess(3)>=180)
    %     q3 = current_guess(3)-180;
    % else
    %     q3 =  current_guess(3);
    % end
    q1 = normalize_angle(current_guess(1)); % Ensure angle is between 0° and 360°
    q2 = normalize_angle(current_guess(2));
    q3 = normalize_angle(current_guess(3));
end



% Corrected Forward Kinematics Function

%matlab
function X = forward_kinematics_func_Eq()
    % Define symbolic joint angles and link lengths
    syms theta1 theta2 theta3
    l1 = 4.045; % Length of link 1
    l2 = 12;    % Length of link 2
    l3 = 21;    % Length of link 3
    
    % Define transformation matrices for each joint using DH parameters
    T1 = transformation_func(theta1, l1, 0, pi/2); % First link
    T2 = transformation_func(theta2, 0, l2, 0);   % Second link
    T3 = transformation_func(theta3, 0, l3, 0);  % Third link
    
    % Compute the total transformation matrix from base to end-effector
    T_total = T1 * T2 * T3;
    
    % Extract the position of the end-effector
    X = T_total(1:3, 4); % [X; Y; Z] position
end


%Transformation Matrix Function

%matlab
function T = transformation_func(theta, d, a, alpha)
    % Compute the transformation matrix for given DH parameters
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

function q_normalized = normalize_angle(q)
    q_normalized = mod(q, 360); % Bring within 0° to 360°
    if q_normalized > 180
        q_normalized = q_normalized - 360; % Adjust to -180° to 180°
    end
end
