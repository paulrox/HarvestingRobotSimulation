function [ q0_dot ] = null_opt(r, type, q_k, con)
%NULL_OPT Returns the q0_dot for null space optimization
%   Returns the vector of the joint space velocities obtained using the
%   optimization specified by 'type'

global dist_grad orient_grad q1 q2 q3 q4 q5 q6 q7 q8

% Objective function to be optimized
if strcmp(type, 'manip')
    % Manipolability
    % obj_f = @(x) -sqrt(det(jacob0(r,x) * jacob0(r,x)'));
    obj_f = @(x) -r.maniplty(x, 'yoshikawa');
    
elseif strcmp(type, 'joint')
    % Distance from mechanical joint limits
    j_mid = mean([r.qlim(:,2) r.qlim(:,1)], 2);
    obj_f = @(x) (1 / (2)) * sumsqr((x' - j_mid) ./ (r.qlim(:,2) - ...
        r.qlim(:,1)));
    
elseif strcmp(type, 'orient')
    % Orientation with the task object
    obj_f = @(x) obj_f_orient(r, x);
    
elseif strcmp(type, 'grad_joint')
    % Distance from mechanical joint limits using gradient
    j_mid = mean([r.qlim(:,1) r.qlim(:,2)], 2);
    g = double(subs(dist_grad,[q1;q2;q3;q4;q5;q6;q7;q8],q_k'))';
    obj_f = @(x) (1 / (2*r.n)) * sumsqr(((q_k - x * g)' - j_mid) ./ ...
            (r.qlim(:,2) - r.qlim(:,1)));
        
elseif strcmp(type, 'grad_orient')
    % Distance from mechanical joint limits using gradient
    g = double(subs(orient_grad,[q1;q2;q3;q4;q5;q6;q7;q8],q_k'))';
    obj_f = @(x) obj_f_orient(r, (q_k - x * g));
else
    error('Undefined optimization type!');
end

% Choose between contrained optimization or not
if strcmp(con, 'yes') 
    % Old constaints, they consider q0
    lb = r.qlim(:, 1)';
    ub = r.qlim(:, 2)';
    
    opt = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'Display', 'off');
    q0_dot = fmincon(obj_f, q_k, [], [], [], [], lb,  ub, [], opt);
    
elseif strcmp(con, 'no')
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
        'Display', 'off');
    q0_dot = fminunc(obj_f, q_k, opt);
    
elseif strcmp(type, 'grad_joint') || strcmp(type, 'grad_orient')
    lb = 0;
    opt = optimoptions('fmincon', 'Display', 'off');
    t = fmincon(obj_f, 1, [], [], [], [], lb,  [], [], opt)
    q0_dot = -t * g;
    
end

end

