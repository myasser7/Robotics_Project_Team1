function T = transformation_func(theta, d, a, alpha)
    % Define the function transformation_func which takes in Denavit-Hartenberg (DH) parameters:
    % theta - joint angle rotation around Z-axis
    % d - link offset along Z-axis
    % a - link length along X-axis
    % alpha - link twist around X-axis
    
    syms theta_sym % Create a symbolic variable theta_sym

    % Check if theta is a symbolic variable
    if ~isa(theta, 'sym') 
        % If theta is not symbolic, convert it to a symbolic variable
        theta_sym = sym(theta);
    else
        % If theta is already symbolic, use it directly
        theta_sym = theta;
    end

    % Construct the 4x4 homogeneous transformation matrix based on DH parameters
    % This matrix represents rotation and translation in 3D space using theta, d, a, and alpha
    T = [cos(theta_sym), -sin(theta_sym)*cos(alpha),  sin(theta_sym)*sin(alpha), a*cos(theta_sym);
         sin(theta_sym),  cos(theta_sym)*cos(alpha), -cos(theta_sym)*sin(alpha), a*sin(theta_sym);
         0,               sin(alpha),                cos(alpha),                d;
         0,               0,                         0,                         1];
     
    % The matrix T is structured as follows:
    % Row 1: X-axis rotation and translation in X direction
    % Row 2: Y-axis rotation and translation in Y direction
    % Row 3: Z-axis rotation and translation in Z direction
    % Row 4: Homogeneous coordinates (1 in the last column)
end

