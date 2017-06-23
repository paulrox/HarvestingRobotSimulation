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

h_robot = SerialLink([platform arm], 'name', 'h_robot');

%figure;

qn = [0 0 qn];

%% Inverse Differential Kinematics

dt = 1;
test_T0 = h_robot.fkine(qn);
test_T1 = trotx(-pi/2) * transl(3, 0, 0);
test_traj = ctraj(test_T0, test_T1, 20);

test_v = zeros(6,1,20);
for i = 1:19
    test_v(:,:,i+1) = (tr2delta(test_traj(:,:,i), test_traj(:,:,i+1)))/dt;
end;

qd = struct;
q = struct;

% null space optimization

qd.no_opt = zeros(1,8,20);
q.no_opt = zeros(21,8);
q.no_opt(1,:) = qn;

for i = 1:20
    J = jacob0(h_robot, qd.no_opt(:,:,i));
    % q0 = null_opt(h_robot, 'manip', q.no_opt(i,:));
    q0 = [0 0 10 10 10 10 10 10]; 
    qd.no_opt(:,:,i) = pinv(J)*test_v(:,:,i) + (eye(8) - pinv(J)*J)*q0';
    q.no_opt(i+1,:) = q.no_opt(i,:) + qd.no_opt(:,:,i)*dt; 
end;

%% Plot the robot

h_robot.plotopt = {'workspace' [-3 3 -3 3 -4 2] 'scale' 0.5, 'jvec'};
h_robot.plot(q.no_opt);

%% Task Objects

% Fruit tree object
Xtree = [-3 -3 -3 -3];
Ytree = [-3 -3 3 3];
Ztree = [-4 2 2 -4];
%fruit_tree = [3 -3 -4; 3 -3 2; -3 -3 2; -3 -3 -4];
%fruit_tree = [-3 -3 -4; -3 -3 2; -3 3 2; -3 3 -4];
fruit_tree = [Xtree; Ytree; Ztree]

% Fruit
C_fruit = [-3; 0; 0.5]
R_fruit = 0.15;

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(C_fruit, R_fruit, 'color', 'r');

%% Building a trajectory
N = 20;
dt = 2;
T0 = robot.fkine(qn);
T1 = transl(1, 0, 2);
TC = ctraj(T0, T1, N);
pi = [];
pi_next = [];
ve = [];

for i = 1: (N - 1)
    tmp = TC(:, :, i);
    tmp = tmp(:, 4);
    for j = 1: 3
        pi(j, :) = tmp(j, :);
    end
    
    tmp = TC(:, :, i + 1);
    tmp = tmp(:, 4);
    for j = 1: 3
        pi_next(j, :) = tmp(j, :);
    end
    
    v = (pi_next - pi) / dt;
    ve(i, :) = [v' 0 0 0];
end

J = jacob0(robot, qn);
qdot = pinv(J) * ve';

% Integration
q(1,:) = qn;
for i = 2: N
    q(i, :) = q(i-1, :) + (qdot(:, i-1)*dt)';
end
