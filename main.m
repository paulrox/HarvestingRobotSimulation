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
robot.qlim = [-2 2;-2 2; deg2rad(110) deg2rad(250); ...
    -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
    -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180); ...
    -deg2rad(360) -deg2rad(270)];
 arm.qlim = [ deg2rad(20) deg2rad(160); ...
     -deg2rad(45) deg2rad(170); deg2rad(140) deg2rad(220); ...
     -deg2rad(170) deg2rad(170); deg2rad(0) deg2rad(180);...
     -deg2rad(360) -deg2rad(270)];

%% Task Objects

% Fruit positions
fruit = cell(1,3);
fruit{1} = [-1.5; -1.5; 0];
fruit{2} = [-1.5; -2.5; 0];
fruit{3} = [-1.5; -1.5; 1.5];

% Workspace limits
min_x = min(scatter(:,1));
min_y = min(scatter(:,2));
min_z = min(scatter(:,3));

max_x = max(scatter(:,1));
max_y = max(scatter(:,2));
max_z = max(scatter(:,3));

%% Pick Phase
% Perform the trajectory planning and inverse kinematics

pick_phase;

%% Place Phase
% Perform the trajectory planning and inverse kinematics

place_phase;

%% Plot the robot in its nominal position for a specific task

plot_robot(robot, qn, fruit{1}, 'r');

%% Plot the robot performing the task

plot_robot(robot, pick{1}.clik.opt{1}.q, fruit{1}, 'r');

%% Plot the robot performing the reachable task without cart 

plot_robot(arm, pick{1}.q_no_cart, fruit{1}, 'r');

%% Workspace analysis

qmin = [20; -45; 140; -170; 0; -360]; 
qmax = [160; 170; 180; 170; 180; 270];

% mcm3;

% hold on;
% for i = 1: length(scatter)
%     if (scatter(i,2) >= 2.5728)
%         break
%     end
% end

% for i = 1: length(work_manipl)
%     if work_manipl(i, :) == 0.0031
%         break
%     end
% end


%% Generate and save the plots

generate_plots;