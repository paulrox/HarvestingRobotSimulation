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

qn = [0 0 qn];

%% Inverse Differential Kinematics

% dt = 0.01;
% n_step = 200;
% % test_T0 = h_robot.fkine(qn);
% test_T0 = p560.fkine(qn);
% % test_T1 = transl(1, -1, -1) * r2t(t2r(test_T0));
% test_T1 = test_T0 * transl(1, 0, 0);
% test_traj = ctraj(test_T0, test_T1, n_step);
% 
% test_v = zeros(6,1,n_step);
% for i = 1 : n_step-1
%     test_v(:,:,i+1) = (tr2delta(test_traj(:,:,i), test_traj(:,:,i+1)))/dt;
% end;
% 
% qd = struct;
% q = struct;
% 
% % null space optimization
% 
% qd.no_opt = zeros(1,6,n_step);
% q.no_opt = zeros(n_step+1,6);
% q.no_opt(1,:,:) = qn;
% 
% J = p560.jacob0(qn);
% for i = 1:n_step
%     % J = h_robot.jacob0(qd.no_opt(:,:,i));
%     % q0 = null_opt(h_robot, 'manip', q.no_opt(i,:));
%     q0 = [0 0 2 2 2 2 2 2];
%     % qns = (eye(8) - pinv(J)*J)*q0';
%     % qd.no_opt(:,:,i) = pinv(J)*test_v(:,:,i) + qns;
%     qd.no_opt(:,:,i) = inv(J)*test_v(:,:,i);
%     J = p560.jacob0(qd.no_opt(:,:,i));
%     q.no_opt(i+1,:) = q.no_opt(i,:) + qd.no_opt(:,:,i)*dt; 
% end;

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

for i = 1: (N)
    if (i ~= N)
        ve(i, :) = tr2delta(TC(:, :, i), TC(:, :, i+1)) / dt;
        
        % No optimization jacobian and joint positions
        no_opt.J = jacob0(robot, no_opt.q(i,:));
        no_opt.qdot(i,:) = pinv(no_opt.J) * ve(i,:)';
        no_opt.q(i+1,:) = no_opt.q(i,:) + (no_opt.qdot(i,:)*dt);
        
        % Null space optimization - Manipulability
        manip.J = jacob0(robot, manip.q(i,:));
        manip.q0 = [0 0 1 1 1 1 1 1];
        manip.qns = (eye(8) - pinv(manip.J) * manip.J) * manip.q0';
        manip.qdot(i,:) = pinv(manip.J) * ve(i,:)' + manip.qns;
        manip.q(i+1,:) = manip.q(i,:) + (manip.qdot(i,:)*dt);
    end   
end

%% Plot the robot

robot.plotopt = {'workspace' [-3 3 -3 3 -4 2] 'scale' 0.5, 'jvec'};
robot.plot(no_opt.q);

%% Workspace analysis
qmin = [-180; -90; -230; -200; -120; -400]; 
qmax = [180; 110; 50; 200; 120; 400];

%%scatter = mcm(arm, qmin, qmax);