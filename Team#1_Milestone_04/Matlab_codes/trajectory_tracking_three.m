% Initialize simulation objects
sim = load_simulation_library(); % Ensure the simulation library is loaded
pi = 3.141592653589793;

% Object initialization (update paths as necessary)
try
    Obj1 = sim.getObject('../q1');
    Obj2 = sim.getObject('../q2');
    Obj3 = sim.getObject('../q3');
    Obj4 = sim.getObject('../q4');
catch exception
    disp('Error: Unable to find simulation objects. Check object paths.');
    disp(getReport(exception));
end

% Joint positions (in radians)
Pos1 = 0 * (pi / 180);
Pos2 = 50 * (pi / 180);
Pos3 = -(-75) * (pi / 180);
Pos4 = (4.878712558 - (-65)) * (pi / 180);

% Simulation duration and counters
req_duration = 5; % seconds
i = 0;
j = 0;
