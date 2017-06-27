function [ q0 ] = null_opt(r, type, q_k, con, k0)
%NULL_OPT Returns the q0 for null space optimization
%   Returns the vector of the joint space velocities obtained using the
%   optimization specified by 'type'

    function dist = obj_f_orient(x)
        obj_z = [1 0 0]';
        kine = r.fkine(x);
        dist = dot(obj_z, kine(1:3,3));
    end
% Objective function to be optimized
if strcmp(type, 'manip')
    % Manipolability
    % obj_f = @(x) -sqrt(det(jacob0(r,x) * jacob0(r,x)'));
    obj_f = @(x) -r.maniplty(x, 'yoshikawa');
    
elseif strcmp(type, 'joint')
    % Distance from mechanical joint limits
    j_mid = mean([r.qlim(:,1) r.qlim(:,2)], 2);
    obj_f = @(x) (1 / r.n) * sumsqr((x - j_mid) ./ (r.qlim(:,2) - ...
        r.qlim(:,1)));
elseif strcmp(type, 'orient')
    % Orientation with the task object
    obj_f = @(x) obj_f_orient(x);
else
    error('Undefined optimization type!');
end

% Choose between contrained optimization or not
if strcmp(con, 'yes')
    
% Old constaints, they consider q0
    lb = r.qlim(:, 1)'
    ub = r.qlim(:, 2)'

%     J = r.jacob0(q_k);
%     Jpinv = J' * ((J * J')^-1);
%     lb = ((r.qlim(:, 1)' -  q_k) / k0 ) / (inv(eye(8) - Jpinv*J))
%     ub = ((r.qlim(:, 2)' -  q_k) / k0 ) / (inv(eye(8) - Jpinv*J))

    opt = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'Display', 'off');
    q0 = fmincon(obj_f, q_k, [], [], [], [], lb,  ub, [], opt);
else
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
        'Display', 'off');
    q0 = fminunc(obj_f, q_k, opt);
    
end

end

