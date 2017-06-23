function [ q0 ] = null_opt(r, type, dq0)
%NULL_OPT Returns the q_0 for null space optimization
%   Returns the vector of the joint space velocities obtained using the
%   optimization specified by 'type'

if type == 'manip'
    opt = optimoptions('fminunc', 'Algorithm', 'quasi-newton');
    obj_f = @(x) sqrt(det(jacob0(r,x) * jacob0(r,x)'));
    q0 = fminunc(obj_f, dq0, opt);
else
    error('Undefined optimization type!');

end

