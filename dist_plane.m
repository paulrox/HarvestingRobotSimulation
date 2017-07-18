function [ d_p ] = dist_plane(r, x )
%dist_plane Squared medium distance from joint positions and tree plane

pjoint = [transl(r.A(3, x')); transl(r.A(4, x')); transl(r.A(5, x')); ...
    transl(r.A(6, x'))];
d_p = sumsqr(pjoint(:,1) + 1.5) / 4;

end
