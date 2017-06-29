%% Place phase

% Tasks definition
pl = cell(1, 3);

T1 = robot.fkine(qn);

% Place phase for each task
pl{1} = struct;
pl{2} = struct;
pl{3} = struct;

%% Cartesian trajectories and inverse differential kinematics

N = 200;
dt = 0.01;
num_opt = 2; % Number of optimizations actually in use

for i = 1 : length(pl)
    disp(['************ TASK ' num2str(i) ' - Place Phase ************']);
    disp('Computing Cartesian trajectory...');
    T0 = transl(task{i}.c_fruit(1), task{i}.c_fruit(2), ...
        task{i}.c_fruit(3)) * robot.base;
    
    pl{i}.TC = ctraj(T0, T1, N);
    pl{i}.ve = zeros(N-1, 6);
    
    % Inverse differential kinematics struct (IK)
    pl{i}.ik = struct;
    % Closed-loop inverse kinematics struct (CLIK)
    pl{i}.clik = struct;
    % Optimization structs
    pl{i}.ik.opt = cell(3,1);
    pl{i}.clik.opt = cell(3,1);
    
    pl{i}.ik.no_opt.q = zeros(N, 8);
    pl{i}.ik.no_opt.q(1,:) = task{i}.ik.no_opt.q(end,:);
    pl{i}.ik.no_opt.qdot = zeros(N-1, 8);
    
    pl{i}.clik.no_opt.q = zeros(N, 8);
    pl{i}.clik.no_opt.q(1,:) = task{i}.clik.no_opt.q(end,:);
    pl{i}.clik.no_opt.qdot = zeros(N-1, 8);
    pl{i}.clik.no_opt.K = eye(6);  % Closed-loop gain matrix
    
    
    disp(['Computing differential IK and CLIK without null space'...
        ' optimizations']);
    for j = 1 : (N-1)
        pl{i}.ve(j,:) = tr2delta(pl{i}.TC(:,:,j), ...
            pl{i}.TC(:,:,j+1)) / dt;
        
        % No null space optimization inverse differential kinematics (IK)
        J = robot.jacob0(pl{i}.ik.no_opt.q(j,:));
        Jpinv = J' * ((J * J')^-1);
        pl{i}.ik.no_opt.qdot(j,:) = Jpinv * pl{i}.ve(j,:)';
        pl{i}.ik.no_opt.q(j+1,:) = pl{i}.ik.no_opt.q(j,:) + ...
            (pl{i}.ik.no_opt.qdot(j,:) * dt);

        % Closed-loop inverse kinematics (CLIK)
        delta_k = tr2delta(robot.fkine(pl{i}.clik.no_opt.q(j,:)), ...
            pl{i}.TC(:,:,j));
        J = robot.jacob0(pl{i}.clik.no_opt.q(j,:));
        Jpinv = J'*((J * J')^-1);
        pl{i}.clik.no_opt.qdot(j,:) = Jpinv * (pl{i}.ve(j,:)' + ...
            pl{i}.clik.no_opt.K * delta_k);
        pl{i}.clik.no_opt.q(j+1,:) = pl{i}.clik.no_opt.q(j,:) + ...
            (pl{i}.clik.no_opt.qdot(j,:) * dt);
    end
end

%% Differential IK and CLIK with null space optimizations

for i = 1 : length(pl)
    disp(['************ TASK ' num2str(i) ' - Place Phase ************']);
    disp('Null Space Optimizations');
    for k = 1 : num_opt
        pl{i}.ik.opt{k}.q = zeros(N,8);
        pl{i}.ik.opt{k}.q(1,:) = task{i}.ik.opt{k}.q(end,:);
        pl{i}.ik.opt{k}.qdot = zeros(N-1,8);
        pl{i}.ik.opt{k}.q0 = zeros(1,8);
        pl{i}.clik.opt{k}.q = zeros(N,8);
        pl{i}.clik.opt{k}.q(1,:) = task{i}.clik.opt{k}.q(end,:);
        pl{i}.clik.opt{k}.qdot = zeros(N-1,8);
        pl{i}.clik.opt{k}.q0 = zeros(1,8);
        pl{i}.clik.opt{k}.K = eye(6);  % Closed-loop gain matrix
        qns = zeros(1,8);
        
        switch k
            case 1
                opt_name = 'joint';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing distance from mechanical joint limits');
            case 2
                opt_name = 'manip';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing manipulability');
            
        end
        
        for j = 1 : (N-1)
            % Null space optimization (IK)
            J = robot.jacob0(pl{i}.ik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            pl{i}.ik.opt{k}.q0 = k0 * null_opt(robot, ...
                opt_name, pl{i}.ik.opt{k}.q(j,:), constraints, k0);
            qns = (eye(8) - Jpinv * J) * pl{i}.ik.opt{k}.q0';
            pl{i}.ik.opt{k}.qdot(j,:) = Jpinv * pl{i}.ve(j,:)'+qns;
            pl{i}.ik.opt{k}.q(j+1,:) = pl{i}.ik.opt{k}.q(j,:) + ...
                (pl{i}.ik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            % check_jlim(robot, task{i}.ik.opt{k}.q(j+1,:));
            
            % Null space optimization (CLIK)
            delta_k = tr2delta(robot.fkine(pl{i}.clik.opt{k}.q(j,:)), ...
                pl{i}.TC(:,:,j));
            J = robot.jacob0(pl{i}.clik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            pl{i}.clik.opt{k}.q0 = k0 * null_opt(robot, opt_name, ...
                pl{i}.clik.opt{k}.q(j,:), constraints, k0);
            qns = (eye(8) - Jpinv * J) * pl{i}.clik.opt{k}.q0';
            pl{i}.clik.opt{k}.qdot(j,:) = Jpinv * (pl{i}.ve(j,:)' + ...
                pl{i}.clik.opt{k}.K * delta_k) + qns;
            pl{i}.clik.opt{k}.q(j+1,:) = pl{i}.clik.opt{k}.q(j,:) + ...
                (pl{i}.clik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            check_jlim(robot, pl{i}.clik.opt{k}.q(j+1,:));
        end
    end
    disp('End of optimization phase');
end

%% Plot the robot performing the task

plot_poly(fruit_tree, 'fill', 'g');
plot_sphere(T1(1:3,4), 0.5, 'color', 'b');
robot.plotopt = {'workspace' [-3 3 -6 4 -4 4] 'scale' 0.7, 'jvec'};
robot.plot(pl{1}.ik.opt{1}.q);
%robot.plot(task{2}.ik.opt{3}.q);

