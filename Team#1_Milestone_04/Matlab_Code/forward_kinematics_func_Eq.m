function X = forward_kinematics_func_Eq(theta_vals)
    % Define symbolic variables for the joint angles (thetas)
    syms theta1 theta2 theta3 

    % Define link lengths for the robotic arm (l1, l2, l3)
    l1 = 4.045; % Length of link 1
    l2 = 12;    % Length of link 2
    l3 = 21;    % Length of link 3

    % Define the DH parameter table
    % Each row corresponds to a joint: [theta, d, a, alpha]
    DH_table = [theta1, l1, 0, pi/2;
                theta2, 0, l2, 0;
                theta3, 0, l3, 0];

    % Initialize the total transformation matrix to the identity matrix
    T_total = eye(4);

    % Loop through each joint to calculate the transformation for each
    for i = 1:size(DH_table, 1)
        % Extract the DH parameters for the current joint (i-th row of DH_table)
        theta = DH_table(i, 1); % Joint angle (theta)
        d = DH_table(i, 2);     % Link offset (d)
        a = DH_table(i, 3);     % Link length (a)
        alpha = DH_table(i, 4); % Link twist angle (alpha)

        % Compute the transformation matrix for the current joint using the DH parameters
        T_i = transformation_func_Eq(theta, d, a, alpha);

        % Update the total transformation matrix
        T_total = T_total * T_i;
    end

    % Extract the position of the end-effector from the transformation matrix
    % The position is located in the 1st to 3rd rows of the 4th column of the matrix
    X_symbolic = T_total(1:3, 4); 

    % If numerical joint angles are provided, calculate the numerical position
    if nargin > 0
        % Substitute the joint angles into the symbolic expression
        X = double(subs(X_symbolic, [theta1, theta2, theta3], theta_vals));
    else
        % If no angles are provided, return the symbolic expression
        X = X_symbolic;
    end
end
