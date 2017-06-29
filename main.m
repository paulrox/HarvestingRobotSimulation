%% Initialize the toolbox
close all;
clearvars;
startup_rvc;

load('scatter.mat');
%% Robot arm configuration

% Arm links

R1 = Link('d', -0.9, 'a', 0.188, 'alpha', -pi/2);
R2 = Link('d', 0, 'a', 0.95, 'alpha', 0);
R3 = Link('d', 0, 'a', 0.225, 'alpha', -pi/2);
R4 = Link('d', -1.705, 'a', 0, 'alpha', pi/2);
R5 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
R6 = Link('d', -0.2, 'a', 0, 'alpha', -pi/2);

arm = SerialLink([R1 R2 R3 R4 R5 R6], 'name', 'arm');

qn = [0 0 3.1416 0.7854 3.1416 0 1.5708 -4.7124];

% Platform links

P1 = Link('theta', 0, 'a', 0.5, 'alpha', -pi/2);
P2 = Link('theta',-pi/2, 'a', 1.5, 'alpha', 0);

platform = SerialLink([P1 P2], 'name', 'platform');
platform.base = trotx(pi/2);


robot = SerialLink([platform arm], 'name', 'h_robot');

% Joint limits
robot.qlim = [-1 1;-1 1; deg2rad(110) deg2rad(250); ...
    -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
    -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180); ...
    -deg2rad(360) -deg2rad(270)];
 arm.qlim = [ deg2rad(20) deg2rad(160); ...
     -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
     -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180);...
     -deg2rad(360) -deg2rad(270)];

%% Task Objects

% Fruit tree object
Xtree = [-1.5 -1.5 -1.5 -1.5];
Ytree = [-6 -6 3 3];
Ztree = [-4 2 2 -4];
fruit_tree = [Xtree; Ytree; Ztree];

% Fruit radius
R_fruit = 0.15;

% Workspace limits
min_x = min(scatter(:,1));
min_y = min(scatter(:,2));
min_z = min(scatter(:,3));

max_x = max(scatter(:,1));
max_y = max(scatter(:,2));
max_z = max(scatter(:,3));

% Tasks definition
task = cell(1, 3);

% Reachable task
task{1} = struct;
task{1}.c_fruit = [-1.5; -1.5; 0];
% Not Reachable tasks, reachable only using cart
task{2} = struct;
task{2}.c_fruit = [-1.5; -2.5; 0];
task{3} = struct;
task{3}.c_fruit = [-1.5; -1.5; 1.5];

%% Cartesian trajectories and inverse differential kinematics

N = 200;
dt = 0.01;
T0 = robot.fkine(qn);
num_opt = 3; % Number of optimizations actually in use

for i = 1 : length(task)
    disp(['************ TASK ' num2str(i) ' - Pick Phase ************']);
    disp('Computing Cartesian trajectory...');
    T1 = transl(task{i}.c_fruit(1), task{i}.c_fruit(2), ...
        task{i}.c_fruit(3))* trotz(-pi/2) * robot.base;
    task{i}.TC = ctraj(T0, T1, N);
    task{i}.ve = zeros(N-1, 6);
    
    % Inverse differential kinematics struct (IK)
    task{i}.ik = struct;
    % Closed-loop inverse kinematics struct (CLIK)
    task{i}.clik = struct;
    % Optimization structs
    task{i}.ik.opt = cell(3,1);
    task{i}.clik.opt = cell(3,1);
    
    task{i}.ik.no_opt.q = zeros(N, 8);
    task{i}.ik.no_opt.q(1,:) = qn;
    task{i}.ik.no_opt.qdot = zeros(N-1, 8);
    
    task{i}.clik.no_opt.q = zeros(N, 8);
    task{i}.clik.no_opt.q(1,:) = qn;
    task{i}.clik.no_opt.qdot = zeros(N-1, 8);
    task{i}.clik.no_opt.K = eye(6);  % Closed-loop gain matrix
    
    
    disp(['Computing differential IK and CLIK without null space'...
        ' optimizations']);
    for j = 1 : (N-1)
        task{i}.ve(j,:) = tr2delta(task{i}.TC(:,:,j), ...
            task{i}.TC(:,:,j+1)) / dt;
        
        % No null space optimization inverse differential kinematics (IK)
        J = robot.jacob0(task{i}.ik.no_opt.q(j,:));
        Jpinv = J' * ((J * J')^-1);
        task{i}.ik.no_opt.qdot(j,:) = Jpinv * task{i}.ve(j,:)';
        task{i}.ik.no_opt.q(j+1,:) = task{i}.ik.no_opt.q(j,:) + ...
            (task{i}.ik.no_opt.qdot(j,:) * dt);

        % Closed-loop inverse kinematics (CLIK)
        delta_k = tr2delta(robot.fkine(task{i}.clik.no_opt.q(j,:)), ...
            task{i}.TC(:,:,j));
        J = robot.jacob0(task{i}.clik.no_opt.q(j,:));
        Jpinv = J'*((J * J')^-1);
        task{i}.clik.no_opt.qdot(j,:) = Jpinv * (task{i}.ve(j,:)' + ...
            task{i}.clik.no_opt.K * delta_k);
        task{i}.clik.no_opt.q(j+1,:) = task{i}.clik.no_opt.q(j,:) + ...
            (task{i}.clik.no_opt.qdot(j,:) * dt);
    end
end

%% Differential IK and CLIK with null space optimizations

for i = 1 : length(task)
    disp(['************ TASK ' num2str(i) ' - Pick Phase ************']);
    disp('Null Space Optimizations');
    for k = 1 : num_opt
        task{i}.ik.opt{k}.q = zeros(N,8);
        task{i}.ik.opt{k}.q(1,:) = qn;
        task{i}.ik.opt{k}.qdot = zeros(N-1,8);
        task{i}.ik.opt{k}.q0 = zeros(1,8);
        task{i}.clik.opt{k}.q = zeros(N,8);
        task{i}.clik.opt{k}.q(1,:) = qn;
        task{i}.clik.opt{k}.qdot = zeros(N-1,8);
        task{i}.clik.opt{k}.q0 = zeros(1,8);
        task{i}.clik.opt{k}.K = eye(6);  % Closed-loop gain matrix
        qns = zeros(1,8);
        
        switch k
            case 1
                opt_name = 'joint';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing distance from mechanical joint limits');
            case 2
                opt_name = 'manip';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing manipulability');
            otherwise
                opt_name  = 'orient';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing orientation with task object');
        end
        
        for j = 1 : (N-1)
            % Null space optimization (IK)
            J = robot.jacob0(task{i}.ik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            task{i}.ik.opt{k}.q0 = k0 * null_opt(robot, ...
                opt_name, task{i}.ik.opt{k}.q(j,:), constraints, k0);
            qns = (eye(8) - Jpinv * J) * task{i}.ik.opt{k}.q0';
            task{i}.ik.opt{k}.qdot(j,:) = Jpinv * task{i}.ve(j,:)'+qns;
            task{i}.ik.opt{k}.q(j+1,:) = task{i}.ik.opt{k}.q(j,:) + ...
                (task{i}.ik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            % check_jlim(robot, task{i}.ik.opt{k}.q(j+1,:));
            
            % Null space optimization (CLIK)
            delta_k = tr2delta(robot.fkine(task{i}.clik.opt{k}.q(j,:)), ...
                task{i}.TC(:,:,j));
            J = robot.jacob0(task{i}.clik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            task{i}.clik.opt{k}.q0 = k0 * null_opt(robot, opt_name, ...
                task{i}.clik.opt{k}.q(j,:), constraints, k0);
            qns = (eye(8) - Jpinv * J) * task{i}.clik.opt{k}.q0';
            task{i}.clik.opt{k}.qdot(j,:) = Jpinv * (task{i}.ve(j,:)' + ...
                task{i}.clik.opt{k}.K * delta_k) + qns;
            task{i}.clik.opt{k}.q(j+1,:) = task{i}.clik.opt{k}.q(j,:) + ...
                (task{i}.clik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            check_jlim(robot, task{i}.clik.opt{k}.q(j+1,:));
        end
    end
    disp('End of optimization phase');
end

%% Trajectory without cart considering joint limits
N = 200;  
T1 = transl(task{1}.c_fruit(1), task{1}.c_fruit(2), ...
        task{1}.c_fruit(3)) * robot.base;
TCR = ctraj(T0, T1, N);

q_no_cart(1,:) = qn(3:8);
for i = 2: N
    q_no_cart(i,:) = arm.ikcon(TCR(:, :, i), q_no_cart(i-1,:));
end

%% Place phase

place;

%% Plot the robot in its nominal position for a specific task

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{1}.c_fruit, R_fruit, 'color', 'r');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(qn);

%% Plot the robot performing the task

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{2}.c_fruit, R_fruit, 'color', 'r');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(task{2}.ik.no_opt.q);
%robot.plot(task{2}.ik.opt{3}.q);
%% Plot the robot performing the reachable task without cart 

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{1}.c_fruit, R_fruit, 'color', 'r');
arm.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
arm.plot(q_no_cart);

%% Workspace analysis

qmin = [20; -45; 140; -170; 0; -360]; 
qmax = [160; 170; 180; 170; 180; 270];
mcm3;

%% Generate and save the plots

generate_plots;

%%
hold on
for i = 1: length(scatter)
    if (scatter(i,2) >= 2.5728)
        break
    end
end
