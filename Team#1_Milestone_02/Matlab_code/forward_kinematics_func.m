function X = forward_kinematics_func(theta11,theta12,theta13);  
    l1 = 4.5;
    l2 = 12.1;
    l3 = 17.72;  
    theta1 = theta11; 
    theta2 = theta12+45; 
    theta3 = theta13-45; 
    
  
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    
    
    T1 = transformation_func(theta1, l1, 1.2, pi/2);
    T2 = transformation_func(theta2, 1.2, l2, 0);
    T3 = transformation_func(theta3, 1.4, l3, 0);
    
    
    T_total = T1 * T2 * T3;
    
    
    X = T_total(1:3, 4); 
end

function T = transformation_func(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end