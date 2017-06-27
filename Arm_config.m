%% Initialize the toolbox
%close all;
%clearvars;
%startup_rvc;

%% Robot arm configuration

% Arm links

R1 = Link('d', -0.9, 'a', 0.188, 'alpha', -pi/2);
R2 = Link('d', 0, 'a', 0.95, 'alpha', 0);
R3 = Link('d', 0, 'a', 0.225, 'alpha', -pi/2);
R4 = Link('d', -1.705, 'a', 0, 'alpha', pi/2);
R5 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
R6 = Link('d', -0.2, 'a', 0, 'alpha', -pi/2);

arm = SerialLink([R1 R2 R3 R4 R5 R6], 'name', 'arm');

qn = [pi 0.7854 3.1416 0 0.7854 0];

% Platform links

P1 = Link('theta', 0, 'a', 0.5, 'alpha', -pi/2);
P2 = Link('theta',-pi/2, 'a', 1.5, 'alpha', 0);

platform = SerialLink([P1 P2], 'name', 'platform');
platform.base = trotx(pi/2);

robot = SerialLink([platform arm], 'name', 'h_robot');

% Joint limits
robot.qlim = [-0.1 0.1;-0.1 0.1; deg2rad(90) deg2rad(270); ...
    -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
    -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180); ...
    -deg2rad(170) deg2rad(170)];

qn = [0 0 qn];

%% Task Objects

% Fruit tree object
Xtree = [-1.5 -1.5 -1.5 -1.5];
Ytree = [-6 -6 3 3];
Ztree = [-4 2 2 -4];
fruit_tree = [Xtree; Ytree; Ztree];

% Fruit radius
R_fruit = 0.15;

% Workspace limits
min_x = -2.37;
min_y = -2.7446;
min_z = -3.4466;

max_x = 2.7446;
max_y = 2.7446;
max_z = 0.7715;

% Task definition
task = cell(1, 1);

% Reachable task
task{1} = struct;
task{1}.c_fruit = [-1.5; -2.5; 0];

%% Cartesian trajectories and inverse differential kinematics

N = 200;
dt = 1;
T0 = robot.fkine(qn);

for i = 1:size(task)
    disp(['************ TASK ' num2str(i) ' ************']);
    disp('Computing Cartesian trajectory...');
    T1 = transl(task{i}.c_fruit(1), task{i}.c_fruit(2), ...
        task{i}.c_fruit(3)) * robot.base;
    task{i}.TC = ctraj(T0, T1, N);
    task{i}.ve = zeros(N-1, 6);
    
    task{i}.no_opt = struct;
    task{i}.opt = cell(2,1);
    
    task{i}.no_opt.q = zeros(N, 8);
    task{i}.no_opt.q(1,:) = qn;
    task{i}.no_opt.qdot = zeros(N-1, 8);
    
    disp(['Computing inverse differential kinematics without null space'...
        ' optimization']);
    for j = 1:(N-1)
        task{i}.ve(j, :) = tr2delta(task{i}.TC(:, :, j), ...
            task{i}.TC(:, :, j+1)) / dt;
        
        % No null space optimization inverse differential kinematics
        task{i}.no_opt.J = robot.jacob0(task{i}.no_opt.q(j,:));
        task{i}.no_opt.Jpinv = task{i}.no_opt.J'*((task{i}.no_opt.J * ...
            task{i}.no_opt.J')^-1);
        task{i}.no_opt.qdot(j,:) = task{i}.no_opt.Jpinv * task{i}.ve(j,:)';
        task{i}.no_opt.q(j+1,:) = task{i}.no_opt.q(j,:) + ...
            (task{i}.no_opt.qdot(j,:)*dt);
    end
    
    disp('Null Space Optimization');
    for k = 1:(size(task{i}.opt, 1))
        task{i}.opt{k}.q = zeros(N,8);
        task{i}.opt{k}.q(1,:) = qn;
        task{i}.opt{k}.qdot = zeros(N-1,8);
        task{i}.opt{k}.q0 = zeros(1,8);
        qns = zeros(1,8);
        
        switch k
            case 1
                opt_name = 'manip';
                constraints = 'yes';
                k0 = 0.03;
                disp('Optimizing manipulability');
            case 2
                opt_name = 'joint';
                constraints = 'no';
                k0 = 0.03;
                disp('Optimizing distance from mechanical joint limits');
            otherwise
                opt_name  = 'obstacle';
                constraints = 'yes';
                k0 = 0.03;
                disp('Optimizing distance from obstacle');
        end
        
        for j = 1:(N-1)
            % Null space optimization - Manipulability
            J = robot.jacob0(task{i}.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            task{i}.opt{k}.q0 = k0 * null_opt(robot, ...
                opt_name, task{i}.no_opt.q(j,:), constraints);
            qns = (eye(8) - Jpinv * J) * task{i}.opt{k}.q0';
            task{i}.opt{k}.qdot(j,:) = Jpinv * task{i}.ve(j,:)' + qns;
            task{i}.opt{k}.q(j+1,:) = task{i}.opt{k}.q(j,:) + ...
                (task{i}.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            check_jlim(robot, task{i}.opt{k}.q(j+1,:));
        end
    end
    
    disp('End of optimization phase');
end

%% Plot the robot

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{1}.c_fruit, R_fruit, 'color', 'r');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(task{1}.opt{2}.q);

%% Workspace analysis
qmin = [-90; -45; 140; -170; 0; -170]; 
qmax = [90; 170; 220; 170; 180; 170];
mcm;
%mcm_script;

