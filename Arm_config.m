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
robot.qlim = [-0.1 0.1;-0.1 0.1; -deg2rad(135) deg2rad(135); ...
    -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
    -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180); ...
    -deg2rad(170) deg2rad(170)];

qn = [0 0 qn];

%% Task Objects

% Fruit tree object
Xtree = [-3 -3 -3 -3];
Ytree = [-3 -3 3 3];
Ztree = [-4 2 2 -4];
%fruit_tree = [3 -3 -4; 3 -3 2; -3 -3 2; -3 -3 -4];
%fruit_tree = [-3 -3 -4; -3 -3 2; -3 3 2; -3 3 -4];
fruit_tree = [Xtree; Ytree; Ztree];

% Fruit
C_fruit = [-3; 0; 0.5];
R_fruit = 0.15;

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(C_fruit, R_fruit, 'color', 'r');

%% Building a trajectory
N = 200;
dt = 1;
T0 = robot.fkine(qn);
T1 = transl(1, 0, 2) * robot.base;
TC = ctraj(T0, T1, N);
ve = zeros(N-1, 6);

no_opt = struct;
manip = struct;

no_opt.q = zeros(N, 8);
no_opt.q(1,:) = qn;
no_opt.qdot = zeros(N-1, 8);

manip.q = zeros(N, 8);
manip.q(1,:) = qn;
manip.qdot = zeros(N-1, 8);
manip.q0 = zeros(1,8);
manip.qns = zeros(1,8);

k0 = 0.03;

for i = 1: (N-1)
        ve(i, :) = tr2delta(TC(:, :, i), TC(:, :, i+1)) / dt;
        
        % No optimization jacobian and joint positions
        no_opt.J = jacob0(robot, no_opt.q(i,:));
        no_opt.Jpinv = no_opt.J'*((no_opt.J*no_opt.J')^-1);
        no_opt.qdot(i,:) = no_opt.Jpinv * ve(i,:)';
        no_opt.q(i+1,:) = no_opt.q(i,:) + (no_opt.qdot(i,:)*dt);
        
        % Null space optimization - Manipulability
        manip.J = jacob0(robot, manip.q(i,:));
        manip.Jpinv = manip.J'*((manip.J*manip.J')^-1);
        manip.q0 = k0*null_opt(robot, 'manip', no_opt.q(i,:), 'no');
        manip.qns = (eye(8) - manip.Jpinv * manip.J) * manip.q0';
        % manip.qns = (null(manip.J))*pinv(null(manip.J)) * manip.q0';
        manip.qdot(i,:) = manip.Jpinv * ve(i,:)' + manip.qns;
        manip.q(i+1,:) = manip.q(i,:) + (manip.qdot(i,:)*dt);   
end

%% Plot the robot

robot.plotopt = {'workspace' [-3 3 -3 3 -4 2] 'scale' 0.5, 'jvec'};
robot.plot(manip.q);
