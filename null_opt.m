function [ q0_dot ] = null_opt(r, type, q_k, options)
%NULL_OPT Returns the q0_dot for null space optimization
%   Returns the vector of the joint space velocities obtained using the
%   optimization specified by 'type'. 'r' is the SerialLink object
%   describing the manipulator, 'q_k' is the starting point for the
%   optimization and 'options' describes the particular optimization method
%   used.

global joint_grad plane_grad q1 q2 q3 q4 q5 q6 q7 q8

%% Objective function to be optimized
switch type

    case 'manip'
    % Manipolability
    obj_f = @(x) -r.maniplty(x, 'yoshikawa');
    
    case 'joint'
    % Distance from mechanical joint limits
    j_mid = mean([r.qlim(:,1) r.qlim(:,2)], 2);
    grad = joint_grad;
    obj_f = @(x) (1 / (2*r.n)) * sumsqr((x' - j_mid) ./ (r.qlim(:,2) - ...
        r.qlim(:,1)));
    
    case 'orient'
    % Orientation with the task object
    obj_f = @(x) obj_f_orient(r, x);
    
    case 'plane'
    % Joint distances from the tree plane
    grad = plane_grad;
    obj_f = @(x) -dist_plane(r, x);
    
    otherwise
    error('Undefined optimization type!');
end

%% Check optimization options

if any(contains(options, 'constrained'))
    % Use joint limits constraints
    % Old constraints, they consider q0
    lb = r.qlim(:, 1)';
    ub = r.qlim(:, 2)';
    
    opt = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'Display', 'off');
    q0_dot = fmincon(obj_f, q_k, [], [], [], [], lb,  ub, [], opt);
    
elseif any(contains(options, 'gradient_fminunc'))
    % Use gradient estimation from fminunc
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
        'Display', 'off', 'MaxIterations', 1);
    [~, ~, ~, ~, q0_dot] = fminunc(obj_f, q_k, opt);
    
elseif any(contains(options, 'gradient_sym'))
    % Use symbolic gradient computed by MATLAB
    g = double(subs(grad,[q1;q2;q3;q4;q5;q6;q7;q8],q_k'))';
    if any(contains(options, 'exact'))
        % Exact line search
        obj_f = @(x) obj_f(q_k - x * g);
        lb = 0;
        opt = optimoptions('fmincon', 'Display', 'off');
        t = fmincon(obj_f, 1, [], [], [], [], lb,  [], [], opt)
        q0_dot = -t * g;
    else
        % Constant step size (k0)
        q0_dot = -g;
    end
    
elseif any(contains(options, 'gradient_est'))
    % Use numeric gradient estimation
    g = grad_est(obj_f, q_k);
    if any(contains(options, 'exact'))
        % Exact line search
        obj_f = @(x) obj_f(q_k - x * g);
        lb = 0;
        opt = optimoptions('fmincon', 'Display', 'iter');
        t = fmincon(obj_f, 1, [], [], [], [], lb,  [], [], opt)
        q0_dot = -t * g;
    else
        % Constant step size (k0)
        q0_dot = -g;
    end
    
else
    % Find the global optimization problem (try to find global optimum)
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
        'Display', 'off');
    [q0_dot] = fminunc(obj_f, q_k, opt);
end

end

