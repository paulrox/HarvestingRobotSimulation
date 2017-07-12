%% Generates and saves the plots needed for the project report

% Check if 'figs/' and its subdirectories exist
if exist('figs', 'dir') ~= 7
    mkdir('figs');
end

gen_pdf = 'no';
fig_path = [pwd '/figs/'];

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
    savefig(fig, [fig_path 'pick' num2str(i) '_traj_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_traj_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
    fig = figure;
    
    R = tr2rpy(pick{i}.TC);
    
    plot(R);
    title(['Task ' num2str(i) ' - Pick - Rotational Trajectory']);
    legend('roll', 'pitch', 'yaw');
    xlabel('Time Steps');
    ylabel('RPY angles [rad]');
    savefig(fig, [fig_path 'pick' num2str(i) '_traj_rot']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_traj_rot'], 'pdf');
    end
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
    savefig(fig, [fig_path 'place' num2str(i) '_traj_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_traj_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
    fig = figure;
    
    R = tr2rpy(place{i}.TC);
    
    plot(R);
    title(['Task ' num2str(i) ' - Place - Rotational Trajectory']);
    legend('roll', 'pitch', 'yaw');
    xlabel('Time Steps');
    ylabel('RPY angles [rad]');
    savefig(fig, [fig_path 'place' num2str(i) '_traj_rot']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_traj_rot'], 'pdf');
    end
    close;
end

%% Trajectory Error IK (Pick Phase)

for i = 1 : length(pick)
    
    % Forward kinematics trajectory (from IK)
    fig = openfig([fig_path 'pick' num2str(i) '_traj_pos']);
    hold on;
    
    P = transl(robot.fkine(pick{i}.ik.no_opt.q(:,:)));
    
    plot(P);
    hold off;
    title(['Task ' num2str(i) ' - Pick - IK Transl. Trajectory Error']);
    legend('Ideal x', 'Ideal y', 'Ideal z', 'IK x', ...
        'IK y', 'IK z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [fig_path 'pick' num2str(i) '_trajVsik_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_trajVsik_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
%     fig = openfig([fig_path 'pick' num2str(i) '_traj_rot']);
%     hold on;
%     
%     R = tr2rpy(robot.fkine(pick{i}.ik.no_opt.q(:,:)));
%     
%     plot(R);
%     hold off;
%     title(['Task ' num2str(i) ' - Pick - IK Rot. Trajectory Error']);
%     legend('Ideal roll', 'Ideal pitch', 'Ideal yaw', 'IK roll', ...
%         'IK pitch', 'IK yaw');
%     xlabel('Time Steps');
%     ylabel('RPY angles [rad]');
%     savefig(fig, [fig_path 'pick' num2str(i) '_trajVsik_rot']);
%     if strcmp(gen_pdf, 'yes')
%         saveas(fig, [fig_path 'pick' num2str(i) '_trajVsik_rot'], 'pdf');
%     end
%     close;
    
end

%% Trajectory Error IK (Place Phase)

for i = 1 : length(place)
    
    % Forward kinematics trajectory (from IK)
    fig = openfig([fig_path 'place' num2str(i) '_traj_pos']);
    hold on;
    
    P = transl(robot.fkine(place{i}.ik.no_opt.q(:,:)));
    
    plot(P);
    hold off;
    title(['Task ' num2str(i) ' - Place - IK Transl. Trajectory Error']);
    legend('Ideal x', 'Ideal y', 'Ideal z', 'IK x', ...
        'IK y', 'IK z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [fig_path 'place' num2str(i) '_trajVsik_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_trajVsik_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
%     fig = openfig([fig_path 'place' num2str(i) '_traj_rot']);
%     hold on;
%     
%     R = tr2rpy(robot.fkine(place{i}.ik.no_opt.q(:,:)));
%     
%     plot(R);
%     hold off;
%     title(['Task ' num2str(i) ' - Place - IK Rot. Trajectory Error']);
%     legend('Ideal roll', 'Ideal pitch', 'Ideal yaw', 'IK roll', ...
%         'IK pitch', 'IK yaw');
%     xlabel('Time Steps');
%     ylabel('RPY angles [rad]');
%     savefig(fig, [fig_path 'place' num2str(i) '_trajVsik_rot']);
%     if strcmp(gen_pdf, 'yes')
%         saveas(fig, [fig_path 'place' num2str(i) '_trajVsik_rot'], 'pdf');
%     end
%     close;
    
end

%% Trajectory Error CLIK (Pick Phase)

for i = 1 : length(pick)
    
    % Forward kinematics trajectory (from CLIK)
    fig = openfig([fig_path 'pick' num2str(i) '_traj_pos']);
    hold on;
    
    P = transl(robot.fkine(pick{i}.clik.no_opt.q(:,:)));
    
    plot(P);
    hold off;
    title(['Task ' num2str(i) ' - Pick - CLIK Transl. Trajectory Error']);
    legend('Ideal x', 'Ideal y', 'Ideal z', 'CLIK x', ...
        'CLIK y', 'CLIK z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [fig_path 'pick' num2str(i) '_trajVsclik_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_trajVsclik_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
%     fig = openfig([fig_path 'pick' num2str(i) '_traj_rot']);
%     hold on;
%     
%     R = tr2rpy(robot.fkine(pick{i}.clik.no_opt.q(:,:)));
%     
%     plot(R);
%     hold off;
%     title(['Task ' num2str(i) ' - Pick - CLIK Rot. Trajectory Error']);
%     legend('Ideal roll', 'Ideal pitch', 'Ideal yaw', 'CLIK roll', ...
%         'CLIK pitch', 'CLIK yaw');
%     xlabel('Time Steps');
%     ylabel('RPY angles [rad]');
%     savefig(fig, [fig_path 'pick' num2str(i) '_trajVsclik_rot']);
%     if strcmp(gen_pdf, 'yes')
%         saveas(fig, [fig_path 'pick' num2str(i) '_trajVsclik_rot'], 'pdf');
%     end
%     close;
    
end

%% Trajectory Error CLIK (Place Phase)

for i = 1 : length(place)
    
    % Forward kinematics trajectory (from CLIK)
    fig = openfig([fig_path 'place' num2str(i) '_traj_pos']);
    hold on;
    
    P = transl(robot.fkine(place{i}.clik.no_opt.q(:,:)));
    
    plot(P);
    hold off;
    title(['Task ' num2str(i) ' - Place - CLIK Transl. Trajectory Error']);
    legend('Ideal x', 'Ideal y', 'Ideal z', 'CLIK x', ...
        'CLIK y', 'CLIK z');
    xlabel('Time Steps');
    ylabel('Distance [m]');
    savefig(fig, [fig_path 'place' num2str(i) '_trajVsclik_pos']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_trajVsclik_pos'], 'pdf');
    end
    close;
    
    % Rotation trajectory (RPY angles)
%     fig = openfig([fig_path 'place' num2str(i) '_traj_rot']);
%     hold on;
%     
%     R = tr2rpy(robot.fkine(place{i}.clik.no_opt.q(:,:)));
%     
%     plot(R);
%     hold off;
%     title(['Task ' num2str(i) ' - Place - CLIK Rot. Trajectory Error']);
%     legend('Ideal roll', 'Ideal pitch', 'Ideal yaw', 'CLIK roll', ...
%         'CLIK pitch', 'CLIK yaw');
%     xlabel('Time Steps');
%     ylabel('RPY angles [rad]');
%     savefig(fig, [fig_path 'place' num2str(i) '_trajVsclik_rot']);
%     if strcmp(gen_pdf, 'yes')
%         saveas(fig, [fig_path 'place' num2str(i) '_trajVsclik_rot'], 'pdf');
%     end
%     close;
    
end

%% Trajectory Error Without Cart (ikcon)
    
% Pick Phase
fig = openfig([fig_path 'pick1_traj_pos']);
hold on;

P = transl(arm.fkine(pick{1}.q_no_cart(:,:)));

plot(P);
hold off;
title('Task 1 (6 DoF) - Pick - ikcon Transl. Trajectory Error');
legend('Ideal x', 'Ideal y', 'Ideal z', 'ikcon x', ...
    'ikcon y', 'ikcon z');
xlabel('Time Steps');
ylabel('Distance [m]');
savefig(fig, [fig_path 'pick1_trajVsikcon_pos']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'pick1_trajVsikcon_pos'], 'pdf');
end
close;

% Plase Phase
fig = openfig([fig_path 'place1_traj_pos']);
hold on;

P = transl(arm.fkine(place{1}.q_no_cart(:,:)));

plot(P);
hold off;
title('Task 1 (6 DoF) - Place - ikcon Transl. Trajectory Error');
legend('Ideal x', 'Ideal y', 'Ideal z', 'ikcon x', ...
    'ikcon y', 'ikcon z');
xlabel('Time Steps');
ylabel('Distance [m]');
savefig(fig, [fig_path 'place1_trajVsikcon_pos']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'place1_trajVsikcon_pos'], 'pdf');
end
close;

%% Ellipsoids

% Pick phase initial pose velocity ellipsoid
% Translational velocity
fig = figure;
grid on;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(1:3, :);

plot_ellipse(J_tmp * J_tmp');
title('Pick - Initial Pose Translational Velocity Ellipsoid');
xlabel('x [m/s]');
ylabel('y [m/s]');
zlabel('z [m/s]');
savefig(fig, [fig_path 'ellips_pick_transl']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'ellips_pick_transl'], 'pdf');
end
close;

% Rotational velocity
fig = figure;
grid on;

J_tmp = robot.jacob0(qn);
J_tmp = J_tmp(4:6, :);

plot_ellipse(J_tmp * J_tmp');
title('Pick - Initial Pose Rotational Velocity Ellipsoid');
xlabel('x [rad/s]');
ylabel('y [rad/s]');
zlabel('z [rad/s]');
savefig(fig, [fig_path 'ellips_pick_rot']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'ellips_pick_rot'], 'pdf');
end
close;

% Place phase initial pose velocity ellipsoids

for i = 1 : length(place)
    % Translational velocity
    fig = figure;
    grid on;
    
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
    savefig(fig, [fig_path 'ellips_place' num2str(i) '_transl']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'ellips_place' num2str(i) '_transl'], 'pdf');
    end
    close;
    
    % Rotational velocity
    fig = figure;
    grid on;
    
    J_tmp = robot.jacob0(place{i}.clik.no_opt.q(end,:));
    J_tmp = J_tmp(4:6, :);
    
    plot_ellipse(J_tmp * J_tmp');
    title(['Task ' num2str(i) ...
        ' - Place - Rotational Velocity Ellipsoid']);
    xlabel('x [rad/s]');
    ylabel('y [rad/s]');
    zlabel('z [rad/s]');
    savefig(fig, [fig_path 'ellips_place' num2str(i) '_rot']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'ellips_place' num2str(i) '_rot'], 'pdf');
    end
    close;

end


%% Joint trajectories (Pick Phase)

for i = 1 : length(pick)
    
    % IK no optimization
    fig = qplot_8dof(pick{i}.ik.no_opt.q);
    
    title(['Task ' num2str(i) ' - Pick IK - No Opt. Joint Space Motion']);
    savefig(fig, [fig_path 'pick' num2str(i) '_ik_noopt_q']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_ik_noopt_q'], 'pdf');
    end
    close;
    
    % CLIK no optimization
    fig = qplot_8dof(pick{i}.clik.no_opt.q);
    
    title(['Task ' num2str(i) ...
        ' -  Pick CLIK - No Opt. Joint Space Motion']);
    savefig(fig, [fig_path 'pick' num2str(i) '_clik_noopt_q']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_clik_noopt_q'], 'pdf');
    end
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
        savefig(fig, [fig_path 'pick' num2str(i) '_ik_' stype '_q']);
        if strcmp(gen_pdf, 'yes')
            saveas(fig, [fig_path 'pick' num2str(i) '_ik_' stype '_q'], 'pdf');
        end
        close;
        
        % CLIK with optimization
        fig = qplot_8dof(pick{i}.clik.opt{j}.q);
        
        title(['Task ' num2str(i) ' CLIK - ' type ' Opt. Joint Space ' ...
            'Motion']);
        savefig(fig, [fig_path 'pick' num2str(i) '_clik_' stype '_q']);
        if strcmp(gen_pdf, 'yes')
            saveas(fig, [fig_path 'pick' num2str(i) '_clik_' stype '_q'], ...
                'pdf');
        end
        close;
        
    end
   
    
end

%% Joint trajectories (Place Phase)

for i = 1 : length(place)
    
    % IK no optimization
    fig = qplot_8dof(place{i}.ik.no_opt.q);
    
    title(['Task ' num2str(i) ' - Place IK - No Opt. Joint Space Motion']);
    savefig(fig, [fig_path 'place' num2str(i) '_ik_noopt_q']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_ik_noopt_q'], 'pdf');
    end
    close;
    
    % CLIK no optimization
    fig = qplot_8dof(place{i}.clik.no_opt.q);
    
    title(['Task ' num2str(i) ...
        ' - Place CLIK - No Opt. Joint Space Motion']);
    savefig(fig, [fig_path 'place' num2str(i) '_clik_noopt_q']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_clik_noopt_q'], 'pdf');
    end
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
        savefig(fig, [fig_path 'place' num2str(i) '_ik_' stype '_q']);
        if strcmp(gen_pdf, 'yes')
            saveas(fig, [fig_path 'place' num2str(i) '_ik_' stype '_q'], ...
                'pdf');
        end
        close;
        
        % CLIK with optimization
        fig = qplot_8dof(place{i}.clik.opt{j}.q);
        
        title(['Task ' num2str(i) ' - Place CLIK - ' type ...
            ' Opt. Joint Space Motion']);
        savefig(fig, [fig_path 'place' num2str(i) '_clik_' stype '_q']);
        if strcmp(gen_pdf, 'yes')
            saveas(fig, [fig_path 'place' num2str(i) '_clik_' stype '_q'], ...
                'pdf');
        end
        close;
        
    end
   
    
end

%% Manipulability analysis CLIK (Pick)

for i = 1 : 1
    
    man_opt = cell(1,3);
    man_noopt = zeros(1,N);
    
    % Manipulability comparison no optimization vs. man. optimization
    for k = 1 : 3
        man_opt{k} = zeros(1,N);
        for j = 1 : N
            man_opt{k}(j) = robot.maniplty(pick{i}.clik.opt{k}.q(j,:));
        end
    end
    
    for j = 1 : N
        man_noopt(j) = robot.maniplty(pick{i}.clik.no_opt.q(j,:));
    end
    
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    for k = 1 : 3
        plot(1:N, man_opt{k});
    end
    title(['Manipulability - Task ' num2str(i) ' - Pick CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. est. exact');
    xlabel('Time Step');
    ylabel('Manipulability value');
    savefig(fig, [fig_path 'pick' num2str(i) '_clik_manip']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_clik_manip'], ...
            'pdf');
    end
    close;
    
end

%% Manipulability analysis CLIK (Place)

for i = 2 : 2
    
    man_opt = cell(1,3);
    man_noopt = zeros(1,N);
    
    % Manipulability comparison no optimization vs. man. optimization
    for k = 1 : 3
        man_opt{k} = zeros(1,N);
        for j = 1 : N
            man_opt{k}(j) = robot.maniplty(place{i}.clik.opt{k}.q(j,:));
        end
    end
    
    for j = 1 : N
        man_noopt(j) = robot.maniplty(place{i}.clik.no_opt.q(j,:));
    end
    
    fig = figure;
    
    plot(1:N, man_noopt);
    hold on;
    for k = 1 : 3
        plot(1:N, man_opt{k});
    end
    title(['Manipulability - Task ' num2str(i) ' - Place CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. est. exact');
    xlabel('Time Step');
    ylabel('Manipulability value');
    savefig(fig, [fig_path 'place' num2str(i) '_clik_manip']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_clik_manip'], ...
            'pdf');
    end
    close;
    
end

%% Joint limits plots CLIK (Pick)

for i = 1 : 1
    
    j_mid = mean([robot.qlim(:,1) robot.qlim(:,2)], 2);
    dist_noopt = zeros(1, N);
    dist_opt = cell(1,4);
    
    for k = 1 : 4
        dist_opt{k} = zeros(1, N);
        for j = 1 : N
            dist_opt{k}(j) = (1 / (2*robot.n)) * sumsqr( ...
                (pick{i}.clik.opt{k}.q(j,:)' - j_mid) ./ (robot.qlim(:,2) - ...
                robot.qlim(:,1)));
        end
    end
    
    for j = 1 : N
        dist_noopt(j) = (1 / (2*robot.n))*sumsqr( ...
            (pick{i}.clik.no_opt.q(j,:)' - j_mid) ./ (robot.qlim(:,2) - ...
            robot.qlim(:,1)));
    end
    
    fig = figure;
    
    plot(1:N, dist_noopt);
    hold on;
    for k = 1 : 4
        plot(1:N, dist_opt{k});
    end
    hold off;
    title(['Joint Medium Distance - Task ' num2str(i) ' - Pick CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. sym', ...
        'Grad. sym exact');
    xlabel('Time Step');
    ylabel('Squared medium distance [m]')
    savefig(fig, [fig_path 'pick' num2str(i) '_clik_dist']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_clik_dist'], 'pdf');
    end
    close;

end

%% Joint limits plots CLIK (Place)

for i = 2 : 2
    
    j_mid = mean([robot.qlim(:,1) robot.qlim(:,2)], 2);
    dist_noopt = zeros(1, N);
    dist_opt = cell(1,3);
    
    for k = 1 : 4
        dist_opt{k} = zeros(1, N);
        for j = 1 : N
            dist_opt{k}(j) = (1 / (2*robot.n)) * sumsqr( ...
                (place{i}.clik.opt{k}.q(j,:)' - j_mid) ./ (robot.qlim(:,2) - ...
                robot.qlim(:,1)));
        end
    end
    
    for j = 1 : N
        dist_noopt(j) = (1 / (2*robot.n))*sumsqr( ...
            (place{i}.clik.no_opt.q(j,:)' - j_mid) ./ (robot.qlim(:,2) - ...
            robot.qlim(:,1)));
    end
    
    fig = figure;
    
    plot(1:N, dist_noopt);
    hold on;
    for k = 1 : 4
        plot(1:N, dist_opt{k});
    end
    hold off;
    title(['Joint Medium Distance - Task ' num2str(i) ' - Place CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. sym', ...
        'Grad. sym exact');
    xlabel('Time Step');
    ylabel('Squared medium distance [m]')
    savefig(fig, [fig_path 'place' num2str(i) '_clik_dist']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_clik_dist'], 'pdf');
    end
    close;

end

%% Plane distance plots CLIK (Pick)

for i = 1 : 1
    
    dist_opt = cell(1,4);
    dist_noopt = zeros(1, N);
    for k = 1 : 4
        dist_opt{k} = zeros(1, N);
        for j = 1 : N
            dist_opt{k}(j) = dist_plane(robot, pick{i}.clik.opt{k}.q(j,:));
        end
    end
      
    for j = 1 : N
        dist_noopt(j) = dist_plane(robot, pick{i}.clik.no_opt.q(j,:));
    end
    
    fig = figure;
    
    plot(1:N, dist_noopt);
    hold on;
    for k = 1 : 4
        plot(1:N, dist_opt{k});
    end
    hold off;
    title(['Plane Medium Distance - Task ' num2str(i) ' - Pick CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. sym', ...
        'Grad. sym exact');
    xlabel('Time Step');
    ylabel('Squared medium distance [m]')
    savefig(fig, [fig_path 'pick' num2str(i) '_clik_plane']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'pick' num2str(i) '_clik_plane'], 'pdf');
    end
    close;

end

%% Plane distance plots CLIK (Place)

for i = 2 : 2
    
    dist_opt = cell(1,4);
    dist_noopt = zeros(1, N);
    for k = 1 : 4
        dist_opt{k} = zeros(1, N);
        for j = 1 : N
            dist_opt{k}(j) = dist_plane(robot, place{i}.clik.opt{k}.q(j,:));
        end
    end
      
    for j = 1 : N
        dist_noopt(j) = dist_plane(robot, place{i}.clik.no_opt.q(j,:));
    end
    
    fig = figure;
    
    plot(1:N, dist_noopt);
    hold on;
    for k = 1 : 4
        plot(1:N, dist_opt{k});
    end
    hold off;
    title(['Plane Medium Distance - Task ' num2str(i) ' - Place CLIK']);
    legend('No opt.', 'fminunc', 'Grad. est.', 'Grad. sym', ...
        'Grad. sym exact');
    xlabel('Time Step');
    ylabel('Squared medium distance [m]')
    savefig(fig, [fig_path 'place' num2str(i) '_clik_plane']);
    if strcmp(gen_pdf, 'yes')
        saveas(fig, [fig_path 'place' num2str(i) '_clik_plane'], 'pdf');
    end
    close;

end

%% Plot the joint positions obtained by q0 for a specific task

q_pos = cell(robot.n,1);
q_pos{1} = zeros(N-1, 3);
q_pos{2} = zeros(N-1, 3);
q_pos{3} = zeros(N-1, 3);
q_pos{4} = zeros(N-1, 3);
q_pos{5} = zeros(N-1, 3);
q_pos{6} = zeros(N-1, 3);
q_pos{7} = zeros(N-1, 3);
q_pos{8} = zeros(N-1, 3);

for i = 1 : N-1
    for j = 1 : robot.n
        q_pos{j}(i, :) = transl(robot.A(j, ...
            pick{1}.clik.opt{1}.q0(i,:)));
    end
end

fig = figure;

hold on;
for i = 1 : robot.n
    scatter3(q_pos{i}(:,1), q_pos{i}(:,2), q_pos{i}(:,3));
end
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8');
plot_robot(robot, qn, fruit{1}, 'r');
hold off;
title('Manip. Optimal Solutions - Task 1 - Pick CLIK');
savefig(fig, [fig_path 'pick1_manip_sol']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'pick1_manip_sol'], 'pdf');
end
close;

%% Plot a specific joint configuration with the robot

q_pos = zeros(robot.n, 3);

for j = 1 : robot.n
    q_pos(j, :) = transl(robot.A(j, ...
        pick{1}.clik.opt{1}.q0(100,:)));
end

fig = figure;

hold on;
for i = 1 : robot.n
    scatter3(q_pos(i,1), q_pos(i,2), q_pos(i,3), '*');
end
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8');
plot_robot(robot, qn, fruit{1}, 'r');
hold off;
title('Joint Distance Optimal Solution - Task 1 - Pick CLIK');
savefig(fig, [fig_path 'pick1_manip_sol']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'pick1_manip_sol'], 'pdf');
end
close;

%% Workspace Analysis plots

% Cloud of points
fig = figure;

scatter3(scatter(:,1), scatter(:,2), scatter(:,3), '.');
hold on

for i = 1: length(work_manipl)
    if (work_manipl(i,1) < 0.5)
        scatter3(scatter(i,1), scatter(i,2), scatter(i,3), '.', 'r');
    end
end

hold off;

savefig(fig, [fig_path 'workspace_cloud']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'workspace_cloud'], ...
        'pdf');
end
close;

% Convex hull
fig = figure;

dela = delaunay(scatter(:,1), scatter(:,2), scatter(:,3));
tsearchn(scatter, dela, [-1.5 -2.5 0]);
trisurf(dela, scatter(:,1),scatter(:,2), scatter(:,3));

savefig(fig, [fig_path 'workspace_convex']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'workspace_convex'], ...
        'pdf');
end
close;

% Workspace manipulability
fig = figure;

histogram(work_manipl(:,1));

savefig(fig, [fig_path 'workspace_manip']);
if strcmp(gen_pdf, 'yes')
    saveas(fig, [fig_path 'workspace_manip'], ...
        'pdf');
end
close;