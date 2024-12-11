%whole trajectory

[q1_1 , q2_1 , q3_1 , t1] =  first_traj();
[q1_4 , q2_4 , q3_4 , t4] =  adjusting_1();
[q1_2 , q2_2 , q3_2 , t2] =  second_traj();
[q1_5 , q2_5 , q3_5 , t5] =  adjusting_2();
[q1_3 , q2_3 , q3_3 , t3] =  third_traj();
t = [t1 , t4 + (5*ones(1,length(t4))) ,  t2 + (6*ones(1,length(t2))) , t5 + (11*ones(1,length(t5)))  t3 + (12*ones(1,length(t3)))];
q1_final = [q1_1 , q1_4 , q1_2 , q1_5 , q1_3];
q2_final = [q2_1 , q2_4 , q2_2 , q2_5 , q2_3];
q3_final = [q3_1 , q3_4 , q3_2 , q3_5 , q3_3];
numSteps1 = length(q1_final); % Ensure same length
 for i = 1:numSteps1
     if q2_final(i) < 0
         q2_final(i) = 180 + q2_final(i);
     end
     if q3_final(i) < 0
         q3_final(i) = 180 + q3_final(i);
     end 
end
q_matrix = [q1_final ; q2_final; q3_final];
q1_series = timeseries(q1_final , t); %first column
q2_series = timeseries(q2_final , t); %second column
q3_series = timeseries(q3_final , t); %third column
%disp(q_matrix);
