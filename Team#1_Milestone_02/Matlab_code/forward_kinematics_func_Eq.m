function X = forward_kinematics_func_Eq()
    syms theta1 theta2 theta3 l1 l2 l3
    
    DH_table = [theta1, l1, 0, pi/2;
                theta2, 0, l2, 0;
                theta3, 0, l3, 0];
    
   
    T_total = eye(4);
    
   
    for i = 1:size(DH_table, 1)
        theta = DH_table(i, 1);
        d = DH_table(i, 2);
        a = DH_table(i, 3);
        alpha = DH_table(i, 4);
        

        T_i = transformation_func(theta, d, a, alpha);
   
        T_total = T_total * T_i;
    end

    X = T_total(1:3, 4); 
end
