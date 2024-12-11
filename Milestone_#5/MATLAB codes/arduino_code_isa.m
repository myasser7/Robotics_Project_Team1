% Check for existing Arduino connection and clear if present



% Load Arduino library
arduinoObj = arduino('COM13', 'Uno', 'Libraries', 'Servo'); % Update COM4 if necessary
% Check for existing Arduino connection and clear if present


% Attach servos to their respective pins
servo1 = servo(arduinoObj, 'D6');   % Servo 1 on pin 6
servo2 = servo(arduinoObj, 'D10');  % Servo 2 on pin 10
servo3 = servo(arduinoObj, 'D11');  % Servo 3 on pin 11
servo4 = servo(arduinoObj, 'D5');   % Servo 4 on pin 5

% Define the q1, q2, and q3 arrays
q1 = q1_final; % Your q1 array values here
q2 = q2_final; % Your q2 array values here
q3 = q3_final; % Your q3 array values here


% Map q1, q2, and q3 values to servo range (0 to 180 degrees)
q1_mapped = q1/180; % Normalize to [0, 1]
q2_mapped = q2/180; % Normalize to [0, 1]
q3_mapped = q3/180; % Normalize to [0, 1]
ef_mapped = 120/180;% Normalize to [0, 1]
% Send values to the servos with a delay
numSteps = length(q1_final); % Ensure same length

writePosition(servo4,ef_mapped); %End Effector gripper

for i = 1:numSteps
    % Write positions to the servos
if q1_mapped(i)<=0
    q1_mapped(i)=0;
end
if q2_mapped(i)<=0
    q2_mapped(i)=0;
end
if q3_mapped(i)<=0
    q3_mapped(i)=0;
end
if(i == 52)
    pause(2);
    writePosition(servo4,0);%End Effector gripper
end
if(i == 125)
    pause(2);
    writePosition(servo4,ef_mapped);%End Effector gripper
end
    writePosition(servo1, q1_mapped(i)); % Value for servo 1 (q1)
    writePosition(servo2, q2_mapped(i)); % Value for servo 2 (q2)
    writePosition(servo3, q3_mapped(i)); % Value for servo 3 (q3)
    % Display values
    fprintf('Servo1 (D6): %.2f, Servo2 (D10): %.2f, Servo3 (D11): %.2f\n, Servo4 (D5): %.2f\n', ...
        q1_mapped(i)*180, q2_mapped(i)*180, q3_mapped(i)*120,ef_mapped*120);

    % Pause for 0.1 seconds
    pause(0.01);
end

disp('Servo movement complete!');