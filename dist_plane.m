function [ d_p ] = dist_plane(r, x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
f = 0;
for i = 3: 6
    pjoint = transl(r.A(i, x));
    f_temp = abs(pjoint(1) + 1.5);
    f = f + f_temp;
end
d_p = f;

end
