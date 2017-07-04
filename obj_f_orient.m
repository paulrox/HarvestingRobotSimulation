function [ dist ] = obj_f_orient( r, x )
%OBJ_F_ORIENT Objective function for orientation optimization
%   Return the scalar

obj_z = [1 0 0]';
kine = r.fkine(x);
dist = dot(obj_z, kine(1:3,3));

end

