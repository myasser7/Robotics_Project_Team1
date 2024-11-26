function T = transformation_func_Eq(theta, d, a, alpha)
    % transformation_func_Eq calculates a Denavit-Hartenberg homogeneous transformation matrix
    % Inputs:
    % theta  - joint angle (rotation about Z-axis)
    % d      - offset along Z-axis (translation)
    % a      - link length along X-axis
    % alpha  - twist (rotation around X-axis)
    
    % Define symbolic variable for theta
    syms theta_sym 
    
    % Check if theta is already symbolic
    if ~isa(theta, 'sym')
        % If not symbolic, convert theta to symbolic
        theta_sym = sym(theta); 
    else
        % If already symbolic, use theta directly
        theta_sym = theta;
    end
    
    % Construct the Denavit-Hartenberg transformation matrix T
    T = [cos(theta_sym), -sin(theta_sym)*cos(alpha),  sin(theta_sym)*sin(alpha), a*cos(theta_sym); 
         sin(theta_sym),  cos(theta_sym)*cos(alpha), -cos(theta_sym)*sin(alpha), a*sin(theta_sym);
         0,               sin(alpha),                cos(alpha),                d;
         0,               0,                         0,                         1];
    % The matrix T represents rotation and translation in 3D space using DH parameters
end
