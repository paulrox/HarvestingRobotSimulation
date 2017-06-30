%% Generates and saves the plots needed for the project report

% Check if the 'plots/' directory exists
if exist('figs', 'dir') ~= 7
    mkdir('figs');
end

path = [pwd '/figs/'];

%% Trajectory plots (Pick Phase)

for i = 1 : length(pick)
    
    % Position trajectory
    fig = figure;
    
    P = transl(pick{i}.TC);
    
    plot(P);
    title(['Task ' num2str(i) ' - Pick - Translational Trajectory']);
    legend('x', 'y', 'z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [path 'pick' num2str(i) '_traj_pos']);
    saveas(fig, [path 'pick' num2str(i) '_traj_pos'], 'pdf');
    close;
    
    % Rotation trajectory (RPY angles)
    fig = figure;
    
    R = tr2rpy(pick{i}.TC);
    
    plot(R);
    title(['Task ' num2str(i) ' - Pick - Rotational Trajectory']);
    legend('roll', 'pitch', 'yaw');
    xlabel('Time Steps');
    ylabel('RPY angles [rad]');
    savefig(fig, [path 'pick' num2str(i) '_traj_rot']);
    saveas(fig, [path 'pick' num2str(i) '_traj_rot'], 'pdf');
    close;
end

%% Trajectory plots (Place Phase)

for i = 1 : length(place)
    
    % Position trajectory
    fig = figure;
    
    P = transl(place{i}.TC);
    
    plot(P);
    title(['Task ' num2str(i) ' - Place - Translational Trajectory']);
    legend('x', 'y', 'z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [path 'place' num2str(i) '_traj_pos']);
    saveas(fig, [path 'place' num2str(i) '_traj_pos'], 'pdf');
    close;
    
    % Rotation trajectory (RPY angles)
    fig = figure;
    
    R = tr2rpy(place{i}.TC);
    
    plot(R);
    title(['Task ' num2str(i) ' - Place - Rotational Trajectory']);
    legend('roll', 'pitch', 'yaw');
    xlabel('Time Steps');
    ylabel('RPY angles [rad]');
    savefig(fig, [path 'place' num2str(i) '_traj_rot']);
    saveas(fig, [path 'place' num2str(i) '_traj_rot'], 'pdf');
    close;
end

%% Ellipsoids

% Pick phase initial pose velocity ellipsoid
% Translational velocity
fig = figure;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(1:3, :);

plot_ellipse(J_tmp * J_tmp');
title('Pick - Initial Pose Translational Velocity Ellipsoid');
xlabel('x [m/s]');
ylabel('y [m/s]');
zlabel('z [m/s]');
savefig(fig, [path 'ellips_pick_transl']);
saveas(fig, [path 'ellips_pick_transl'], 'pdf');
close;

% Rotational velocity
fig = figure;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(4:6, :);

plot_ellipse(J_tmp * J_tmp');
title('Pick - Initial Pose Rotational Velocity Ellipsoid');
xlabel('x [rad/s]');
ylabel('y [rad/s]');
zlabel('z [rad/s]');
savefig(fig, [path 'ellips_pick_rot']);
saveas(fig, [path 'ellips_pick_rot', 'pdf']);
close;

% Place phase initial pose velocity ellipsoids

for i = 1 : length(place)
    % Translational velocity
    fig = figure;
    
    % We consider the value of the joint variables for the closed-loop
    % inverse kinematics because gives the lowest error
    J_tmp = robot.jacob0(place{i}.clik.no_opt.q(end,:));
    J_tmp = J_tmp(1:3, :);
    
    plot_ellipse(J_tmp * J_tmp');
    title(['Task ' num2str(i) ...
        ' - Place - Translational Velocity Ellipsoid']);
    xlabel('x [m/s]');
    ylabel('y [m/s]');
    zlabel('z [m/s]');
    savefig(fig, [path 'ellips_place' num2str(i) '_transl']);
    saveas(fig, [path 'ellips_place' num2str(i) '_transl'], 'pdf');
    close;
    
    % Rotational velocity
    fig = figure;
    
    J_tmp = robot.jacob0(place{i}.clik.no_opt.q(end,:));
    J_tmp = J_tmp(4:6, :);
    
    plot_ellipse(J_tmp * J_tmp');
    title(['Task ' num2str(i) ...
        ' - Place - Rotational Velocity Ellipsoid']);
    xlabel('x [rad/s]');
    ylabel('y [rad/s]');
    zlabel('z [rad/s]');
    savefig(fig, [path 'ellips_place' num2str(i) '_rot']);
    saveas(fig, [path 'ellips_place' num2str(i) '_rot', 'pdf']);
    close;

end


%% Joint trajectories (Pick Phase)

for i = 1 : length(pick)
    
    % IK no optimization
    fig = qplot_8dof(pick{i}.ik.no_opt.q);
    
    title(['Task ' num2str(i) ' - Pick IK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'pick' num2str(i) '_ik_noopt_q']);
    saveas(fig, [path 'pick' num2str(i) '_ik_noopt_q'], 'pdf');
    close;
    
    % CLIK no optimization
    fig = qplot_8dof(pick{i}.clik.no_opt.q);
    
    title(['Task ' num2str(i) ...
        ' -  Pick CLIK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'pick' num2str(i) '_clik_noopt_q']);
    saveas(fig, [path 'pick' num2str(i) '_clik_noopt_q'], 'pdf');
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
        fig = qplot_8dof(pick{i}.ik.opt{j}.q);
        
        title(['Task ' num2str(i) ' Pick IK - ' type ...
            ' Opt. Joint Space Motion']);
        savefig(fig, [path 'pick' num2str(i) '_ik_' stype '_q']);
        saveas(fig, [path 'pick' num2str(i) '_ik_' stype '_q'], 'pdf');
        close;
        
        % CLIK with optimization
        fig = qplot_8dof(pick{i}.clik.opt{j}.q);
        
        title(['Task ' num2str(i) ' CLIK - ' type ' Opt. Joint Space ' ...
            'Motion']);
        savefig(fig, [path 'pick' num2str(i) '_clik_' stype '_q']);
        saveas(fig, [path 'pick' num2str(i) '_clik_' stype '_q'], 'pdf');
        close;
        
    end
   
    
end

%% Joint trajectories (Place Phase)

for i = 1 : length(place)
    
    % IK no optimization
    fig = qplot_8dof(place{i}.ik.no_opt.q);
    
    title(['Task ' num2str(i) ' - Place IK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'place' num2str(i) '_ik_noopt_q']);
    saveas(fig, [path 'place' num2str(i) '_ik_noopt_q'], 'pdf');
    close;
    
    % CLIK no optimization
    fig = qplot_8dof(place{i}.clik.no_opt.q);
    
    title(['Task ' num2str(i) ...
        ' - Place CLIK - No Opt. Joint Space Motion']);
    savefig(fig, [path 'place' num2str(i) '_clik_noopt_q']);
    saveas(fig, [path 'place' num2str(i) '_clik_noopt_q'], 'pdf');
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
        fig = qplot_8dof(place{i}.ik.opt{j}.q);
        
        title(['Task ' num2str(i) ' - Place IK - ' type ...
            ' Opt. Joint Space Motion']);
        savefig(fig, [path 'place' num2str(i) '_ik_' stype '_q']);
        saveas(fig, [path 'place' num2str(i) '_ik_' stype '_q'], 'pdf');
        close;
        
        % CLIK with optimization
        fig = qplot_8dof(place{i}.clik.opt{j}.q);
        
        title(['Task ' num2str(i) ' - Place CLIK - ' type ...
            ' Opt. Joint Space Motion']);
        savefig(fig, [path 'place' num2str(i) '_clik_' stype '_q']);
        saveas(fig, [path 'place' num2str(i) '_clik_' stype '_q'], 'pdf');
        close;
        
    end
   
    
end

%% Manipulability analysis (Pick)

for i = 1 : length(pick)
    
    % Manipulability comparison IK
    man_noopt = zeros(1,N);
    man_opt = zeros(1,N);
    
    for j = 1 : N
        man_noopt(j) = robot.maniplty(pick{i}.ik.no_opt.q(j,:));
        man_opt(j) = robot.maniplty(pick{i}.ik.opt{2}.q(j,:));
    end
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    plot(1:N, man_opt);
    hold off;
    title(['Manipulability - Task ' num2str(i) ' - Pick IK']);
    legend('No opt.', 'Opt.');
    savefig(fig, [path 'pick' num2str(i) '_ik_manip']);
    saveas(fig, [path 'pick' num2str(i) '_ik_manip'], 'pdf');
    close;
    
    % Manipulability comparison CLIK
    for j = 1 : N
        man_noopt(j) = robot.maniplty(pick{i}.clik.no_opt.q(j,:));
        man_opt(j) = robot.maniplty(pick{i}.clik.opt{2}.q(j,:));
    end 
    
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    plot(1:N, man_opt);
    title(['Manipulability - Task ' num2str(i) ' - Pick CLIK']);
    legend('No opt.', 'Opt.');
    savefig(fig, [path 'pick' num2str(i) '_clik_manip']);
    saveas(fig, [path 'pick' num2str(i) '_clik_manip'], ...
        'pdf');
    close;
    
end

%% Manipulability analysis (Place)

for i = 1 : length(place)
    
    % Manipulability comparison IK
    man_noopt = zeros(1,N);
    man_opt = zeros(1,N);
    
    for j = 1 : N
        man_noopt(j) = robot.maniplty(place{i}.ik.no_opt.q(j,:));
        man_opt(j) = robot.maniplty(place{i}.ik.opt{2}.q(j,:));
    end
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    plot(1:N, man_opt);
    hold off;
    title(['Manipulability - Task ' num2str(i) ' - Place IK']);
    legend('No opt.', 'Opt.');
    savefig(fig, [path 'place' num2str(i) '_ik_manip']);
    saveas(fig, [path 'place' num2str(i) '_ik_manip'], 'pdf');
    close;
    
    % Manipulability comparison CLIK
    for j = 1 : N
        man_noopt(j) = robot.maniplty(place{i}.clik.no_opt.q(j,:));
        man_opt(j) = robot.maniplty(place{i}.clik.opt{2}.q(j,:));
    end 
    
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    plot(1:N, man_opt);
    title(['Manipulability - Task ' num2str(i) ' - Place CLIK']);
    legend('No opt.', 'Opt.');
    savefig(fig, [path 'place' num2str(i) '_clik_manip']);
    saveas(fig, [path 'place' num2str(i) '_clik_manip'], ...
        'pdf');
    close;
    
end

%% Workspace Analysis plots

scatter3(scatter(:,1), scatter(:,2), scatter(:,3), '.');

hold on
for i = 1: length(work_manipl)
    if (work_manipl(i,1) < 0.5)
        scatter3(scatter(i,1), scatter(i,2), scatter(i,3), '.', 'r');
    end
end


dela = delaunay(scatter(:,1), scatter(:,2), scatter(:,3));
tsearchn(scatter, dela, [-1.5 -2.5 0])
trisurf(dela, scatter(:,1),scatter(:,2), scatter(:,3))

h = histogram(work_manipl(:,1));
for i = 1: length(work_manipl)
    if work_manipl(i, :) == 0.0031
        break
    end
end
