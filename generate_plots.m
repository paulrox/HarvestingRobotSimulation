%% Generates and saves the plots needed for the project report

% Check if the 'plots/' directory exists
if exist('figs', 'dir') ~= 7
    mkdir('figs');
end

path = [pwd '/figs/'];

%% Trajectory plots

for i = 1 : size(task)
    
    % Position trajectory
    fig = figure;
    
    P = transl(task{i}.TC);
    
    plot(P);
    title(['Task ' num2str(i) ' Translational Trajectory']);
    legend('x', 'y', 'z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [path 'task' num2str(i) '_traj_pos']);
    saveas(fig, [path 'task' num2str(i) '_traj_pos'], 'pdf');
    close;
    
    % Rotation trajectory (RPY angles)
    fig = figure;
    
    R = tr2rpy(task{i}.TC);
    
    plot(R);
    title(['Task ' num2str(i) ' Translational Trajectory']);
    legend('roll', 'pitch', 'yaw');
    xlabel('Time Steps');
    ylabel('RPY angles [rad]');
    savefig(fig, [path 'task' num2str(i) '_traj_rot']);
    saveas(fig, [path 'task' num2str(i) '_traj_rot'], 'pdf');
    close;
end

%% Ellipsoids

% Initial pose velocity ellipsoid
% Translational velocity
fig = figure;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(1:3, :);

plot_ellipse(J_tmp * J_tmp');
title('Initial Pose Translational Velocity Ellipsoid');
xlabel('x [m/s]');
ylabel('y [m/s]');
zlabel('z [m/s]');
savefig(fig, [path 'ellips_qn_transl']);
saveas(fig, [path 'ellips_qn_transl'], 'pdf');
close;

% Rotational velocity
fig = figure;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(4:6, :);

plot_ellipse(J_tmp * J_tmp');
title('Initial Pose Rotational Velocity Ellipsoid');
xlabel('x [rad/s]');
ylabel('y [rad/s]');
zlabel('z [rad/s]');
savefig(fig, [path 'ellips_qn_rot']);
saveas(fig, [path 'ellips_qn_rot', 'pdf']);
close;

%% Joint trajectories

for i = 1 : size(task)
    
    % IK no optimization
    fig = qplot_8dof(task{i}.ik.no_opt.q);
    title(['Task ' num2str(i) ' IK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'task' num2str(i) '_ik_noopt_q']);
    saveas(fig, [path 'task' num2str(i) '_ik_noopt_q'], 'pdf');
    close;
    
    % CLIK no optimization
    fig = qplot_8dof(task{i}.clik.no_opt.q);
    title(['Task ' num2str(i) ' CLIK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'task' num2str(i) '_clik_noopt_q']);
    saveas(fig, [path 'task' num2str(i) '_clik_noopt_q'], 'pdf');
    close;
    
    % Optimizations
    for j = 1 : num_opt
        switch j
            case 1
                type = 'Manipulability';
                stype = 'manip';
            case 2
                type = 'Joint Limits';
                stype = 'jlim';
            otherwise
                type = 'Orientation';
                stype = 'orient';
        end
        
        % IK with optimization
        fig = qplot_8dof(task{i}.ik.opt{j}.q);
        title(['Task ' num2str(i) ' IK - ' type ' Opt. Joint Space ' ...
            'Motion']);
        savefig(fig, [path 'task' num2str(i) '_ik_' stype '_q']);
        saveas(fig, [path 'task' num2str(i) '_ik_' stype '_q'], 'pdf');
        close;
        
        % CLIK with optimization
        fig = qplot_8dof(task{i}.clik.opt{j}.q);
        title(['Task ' num2str(i) ' CLIK - ' type ' Opt. Joint Space ' ...
            'Motion']);
        savefig(fig, [path 'task' num2str(i) '_clik_' stype '_q']);
        saveas(fig, [path 'task' num2str(i) '_clik_' stype '_q'], 'pdf');
        close;
        
    end
   
    
end

%% Manipulability analysis

h = histogram(work_manipl(:,1));


for i = 1: length(work_manipl)
    if work_manipl(i, :) == 0.0031
        break
    end
end

%%

scatter3(scatter(:,1), scatter(:,2), scatter(:,3), '.')
%%
hold on
for i = 1: length(work_manipl)
    if (work_manipl(i,1) < 0.5)
        scatter3(scatter(i,1), scatter(i,2), scatter(i,3), '.', 'r');
    end
end

%%
dela = delaunay(scatter(:,1), scatter(:,2), scatter(:,3));
tsearchn(scatter, dela, [-1.5 -2.5 0])
trisurf(dela, scatter(:,1),scatter(:,2), scatter(:,3))
