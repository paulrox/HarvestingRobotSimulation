%% Gradient analytical definitions for the objective functions

global dist_grad plane_grad q1 q2 q3 q4 q5 q6 q7 q8 q_sym

% Joint variables
syms q1 q2 q3 q4 q5 q6 q7 q8 real
q_sym = [q1 q2 q3 q4 q5 q6 q7 q8];

% Distance from joint limits
j_mid = mean([robot.qlim(:,1) robot.qlim(:,2)], 2);
dist_grad = gradient((1 / (2*robot.n)) * sumsqr((q_sym' - j_mid) ./ ...
    (robot.qlim(:,2) - robot.qlim(:,1))));

% Tree plane distance
plane_grad = gradient(dist_plane(robot, q_sym'));
plane_grad = [0; 0; plane_grad; 0; 0; 0];
