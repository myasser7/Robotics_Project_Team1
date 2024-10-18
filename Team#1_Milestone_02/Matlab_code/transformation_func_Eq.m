function T = transformation_func_Eq(theta, d, a, alpha)
    
    syms theta_sym 
    if ~isa(theta, 'sym')
        theta_sym = sym(theta); 
    else
        theta_sym = theta;
    end
    
 
    T = [cos(theta_sym), -sin(theta_sym)*cos(alpha),  sin(theta_sym)*sin(alpha), a*cos(theta_sym);
         sin(theta_sym),  cos(theta_sym)*cos(alpha), -cos(theta_sym)*sin(alpha), a*sin(theta_sym);
         0,               sin(alpha),               cos(alpha),               d;
         0,               0,                         0,                         1];
end