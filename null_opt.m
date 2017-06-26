function [ q0 ] = null_opt(r, type, q_k, con)
%NULL_OPT Returns the q0 for null space optimization
%   Returns the vector of the joint space velocities obtained using the
%   optimization specified by 'type'

if strcmp(con, 'yes')
    lb = r.qlim(:, 1)';
    ub = r.qlim(:, 2)';

    opt = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
            'Display', 'off');
else
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton', ...
            'Display', 'off');
end
    

if type == 'manip'
    % obj_f = @(x) -sqrt(det(jacob0(r,x) * jacob0(r,x)'));
    obj_f = @(x) -r.maniplty(x, 'yoshikawa');

else
    error('Undefined optimization type!');
end

if strcmp(con, 'yes')
    q0 = fmincon(obj_f, q_k, [], [], [], [], lb,  ub, [], opt);
else
    q0 = fminunc(obj_f, q_k, opt);

end

