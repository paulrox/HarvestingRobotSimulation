%% Initialize the toolbox
%close all;
%clearvars;
%startup_rvc;

%% Robot arm configuration

% Loading cloud of workload points
load('scatter.mat');

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

qmin = [-90; -45; 140; -170; 0; -170]; 
qmax = [270; 170; 220; 170; 180; 170];

robot.qlim = [-0.1 0.1;-0.1 0.1; -deg2rad(90) deg2rad(270); ...
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


%% Building a trajectory

N = 200;
dt = 1;
k0 = 0.015;
T0 = robot.fkine(qn);
for i = 1:length(task)
  
    T1 = transl(task{i}.c_fruit(1), task{i}.c_fruit(2), ...
        task{i}.c_fruit(3)) * robot.base;
    task{i}.TC = ctraj(T0, T1, N);
    task{i}.ve = zeros(N-1, 6);
    
    task{i}.no_opt = struct;
    task{i}.manip = struct;
    
    task{i}.no_opt.q = zeros(N, 8);
    task{i}.no_opt.q(1,:) = qn;
    task{i}.no_opt.qdot = zeros(N-1, 8);
    
    task{i}.manip.q = zeros(N, 8);
    task{i}.manip.q(1,:) = qn;
    task{i}.manip.qdot = zeros(N-1, 8);
    task{i}.manip.q0 = zeros(1,8);
    task{i}.manip.qns = zeros(1,8);
    
    for j = 1: (N-1)
        task{i}.ve(j, :) = tr2delta(task{i}.TC(:, :, j), ...
            task{i}.TC(:, :, j+1)) / dt;
        
        % No optimization jacobian and joint positions
        task{i}.no_opt.J = robot.jacob0(task{i}.no_opt.q(j,:));
        task{i}.no_opt.Jpinv = task{i}.no_opt.J'*((task{i}.no_opt.J * ...
            task{i}.no_opt.J')^-1);
        task{i}.no_opt.qdot(j,:) = task{i}.no_opt.Jpinv * task{i}.ve(j,:)';
        task{i}.no_opt.q(j+1,:) = task{i}.no_opt.q(j,:) + ...
            (task{i}.no_opt.qdot(j,:)*dt);
        
        % Null space optimization - Manipulability
        task{i}.manip.J = robot.jacob0(task{i}.manip.q(j,:));
        task{i}.manip.Jpinv = task{i}.manip.J'*((task{i}.manip.J * ...
            task{i}.manip.J')^-1);
        task{i}.manip.q0 = k0*null_opt(robot, 'manip', ...
            task{i}.no_opt.q(j,:), 'yes');
        task{i}.manip.qns = (eye(8) - task{i}.manip.Jpinv * ...
            task{i}.manip.J) * task{i}.manip.q0';
        % manip.qns = (null(manip.J))*pinv(null(manip.J)) * manip.q0';
        task{i}.manip.qdot(j,:) = task{i}.manip.Jpinv * ...
            task{i}.ve(j,:)' + task{i}.manip.qns;
        task{i}.manip.q(j+1,:) = task{i}.manip.q(j,:) + ...
            (task{i}.manip.qdot(j,:)*dt);
      
    end
end

%% Trajectory without cart considering joint limits
N = 200;  
T1 = transl(task{1}.c_fruit(1), task{i}.c_fruit(2), ...
        task{i}.c_fruit(3)) * robot.base;
TCR = ctraj(T0, T1, N);

q_no_cart(1,:) = qn(3:8);
for i = 2: N
    q_no_cart(i,:) = arm.ikcon(TCR(:, :, i), q_no_cart(i-1,:));
end

    
%% Plot the robot in its nominal position for a specific task

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{1}.c_fruit, R_fruit, 'color', 'r');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(qn);

%% Plot the robot performing the task

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{3}.c_fruit, R_fruit, 'color', 'r');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(task{3}.manip.q);

%% Plot the robot performing the reachable task without cart 

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(task{1}.c_fruit, R_fruit, 'color', 'r');
arm.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
arm.plot(q_no_cart)

%% Workspace analysis

qmin = [-90; -45; 140; -170; 0; -170]; 
qmax = [270; 170; 180; 170; 180; 170];
mcm3;
%% Manipulability analysis

h = histogram(work_manipl(:,1))


for (i = 1: length(work_manipl))
    if work_manipl(i, :) == 0.0031
        break
    end
end
%%
scatter3(scatter(:,1), scatter(:,2), scatter(:,3), '.')
hold on
for (i = 1: length(work_manipl))
    if (work_manipl(i,1) < 0.5)
        scatter3(scatter(i,1), scatter(i,2), scatter(i,3), '.', 'r') 
    end
end

%%
dela = delaunay(scatter(:,1), scatter(:,2), scatter(:,3));
tsearchn(scatter, dela, [-1.5 -2.5 0])
trisurf(dela, scatter(:,1),scatter(:,2), scatter(:,3))

