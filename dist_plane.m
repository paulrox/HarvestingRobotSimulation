function [ d_p ] = dist_plane(r, x )
%dist_plane Squared medium distance from joint positions and tree plane
% f = 0;
% for i = 3: 6
%     pjoint = transl(r.A(i, x'));
%     f_temp = abs(pjoint(1) + 1.5);
%     f = f + f_temp;
% end
% d_p = f;

pjoint = [transl(r.A(3, x')); transl(r.A(4, x')); transl(r.A(5, x')); ...
    transl(r.A(6, x'))];
d_p = sumsqr(pjoint(:,1) + 1.5) / 4;

end
