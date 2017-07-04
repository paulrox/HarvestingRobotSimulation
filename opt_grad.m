%% Optimization using gradient

global dist_grad orient_grad q1 q2 q3 q4 q5 q6 q7 q8 q_sym

syms q1 q2 q3 q4 q5 q6 q7 q8 real

q_sym = [q1 q2 q3 q4 q5 q6 q7 q8];

j_mid = mean([robot.qlim(:,1) robot.qlim(:,2)], 2);

dist_grad = gradient((1 / (2*robot.n)) * sumsqr((q_sym' - j_mid) ./ (robot.qlim(:,2) - ...
robot.qlim(:,1))));

orient_grad = gradient(obj_f_orient(robot, q_sym'));

% manip_grad = gradient(sqrt(det(jacob0(robot,q_sym) * jacob0(robot,q_sym)')));
% cond_grad = gradient(cond(robot.jacob0(q_sym)));