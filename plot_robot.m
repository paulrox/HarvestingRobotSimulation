function plot_robot( r, q, pos, color)
%PLOT_ROBOT Plot the harvesting robot with the task object
%   Plot the harvesting robot with the fruit tree and the task object

% Fruit tree object
Xtree = [-1.5 -1.5 -1.5 -1.5];
Ytree = [-6 -6 3 3];
Ztree = [-4 2 2 -4];
fruit_tree = [Xtree; Ytree; Ztree];

% Fruit radius
R_fruit = 0.15;

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(pos, R_fruit, 'color', color);
r.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
r.plot(q);

end

