%% Pick Phase

% Compute or not optimizations for open-loop IK
use_ik_opt = 0;

% Tasks definition
pick = cell(1, 3);

% Pick phase for each task
pick{1} = struct;
pick{2} = struct;
pick{3} = struct;

%% Cartesian trajectories and inverse differential kinematics

N = 200;
dt = 0.01;
T0 = robot.fkine(qn);
num_opt = 1; % Number of optimizations actually in use

for i = 1 : length(pick)
    disp(['************ TASK ' num2str(i) ' - Pick Phase ************']);
    disp('Computing Cartesian trajectory...');
    T1 = transl(fruit{i}(1), fruit{i}(2), fruit{i}(3))* trotz(-pi/2) ...
        * robot.base;
    pick{i}.TC = ctraj(T0, T1, N);
    pick{i}.ve = zeros(N-1, 6);
    
    % Inverse differential kinematics struct (IK)
    pick{i}.ik = struct;
    % Closed-loop inverse kinematics struct (CLIK)
    pick{i}.clik = struct;
    % Optimization structs
    pick{i}.ik.opt = cell(3,1);
    pick{i}.clik.opt = cell(3,1);
    
    pick{i}.ik.no_opt.q = zeros(N, 8);
    pick{i}.ik.no_opt.q(1,:) = qn;
    pick{i}.ik.no_opt.qdot = zeros(N-1, 8);
    
    pick{i}.clik.no_opt.q = zeros(N, 8);
    pick{i}.clik.no_opt.q(1,:) = qn;
    pick{i}.clik.no_opt.qdot = zeros(N-1, 8);
    pick{i}.clik.no_opt.K = eye(6);  % Closed-loop gain matrix
    
    
    disp(['Computing differential IK and CLIK without null space'...
        ' optimizations']);
    for j = 1 : (N-1)
        pick{i}.ve(j,:) = tr2delta(pick{i}.TC(:,:,j), ...
            pick{i}.TC(:,:,j+1)) / dt;
        
        % No null space optimization inverse differential kinematics (IK)
        J = robot.jacob0(pick{i}.ik.no_opt.q(j,:));
        Jpinv = J' * ((J * J')^-1);
        pick{i}.ik.no_opt.qdot(j,:) = Jpinv * pick{i}.ve(j,:)';
        pick{i}.ik.no_opt.q(j+1,:) = pick{i}.ik.no_opt.q(j,:) + ...
            (pick{i}.ik.no_opt.qdot(j,:) * dt);

        % Closed-loop inverse kinematics (CLIK)
        delta_k = tr2delta(robot.fkine(pick{i}.clik.no_opt.q(j,:)), ...
            pick{i}.TC(:,:,j));
        J = robot.jacob0(pick{i}.clik.no_opt.q(j,:));
        Jpinv = J'*((J * J')^-1);
        pick{i}.clik.no_opt.qdot(j,:) = Jpinv * (pick{i}.ve(j,:)' + ...
            pick{i}.clik.no_opt.K * delta_k);
        pick{i}.clik.no_opt.q(j+1,:) = pick{i}.clik.no_opt.q(j,:) + ...
            (pick{i}.clik.no_opt.qdot(j,:) * dt);
        check_jlim(robot, pick{i}.clik.no_opt.q(j+1,:));
    end
end

%% Differential IK and CLIK with null space optimizations

for i = 1 : length(pick)
    disp(['************ TASK ' num2str(i) ' - Pick Phase ************']);
    disp('Null Space Optimizations');
    for k = 1 : num_opt
        pick{i}.ik.opt{k}.q = zeros(N,8);
        pick{i}.ik.opt{k}.q(1,:) = qn;
        pick{i}.ik.opt{k}.qdot = zeros(N-1,8);
        pick{i}.ik.opt{k}.q0 = zeros(1,8);
        pick{i}.clik.opt{k}.q = zeros(N,8);
        pick{i}.clik.opt{k}.q(1,:) = qn;
        pick{i}.clik.opt{k}.qdot = zeros(N-1,8);
        pick{i}.clik.opt{k}.q0 = zeros(1,8);
        pick{i}.clik.opt{k}.K = eye(6);  % Closed-loop gain matrix
        qns = zeros(1,8);
        
        switch k
            case 2
                opt_name = 'joint';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing distance from mechanical joint limits');
            case 4
                opt_name = 'grad_joint';
                constraints = 'ciao';
                k0 = 1;
                disp('Optimizing joint distance using gradient');
            case 5
                opt_name = 'manip';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing manipulability');
            case 3
                opt_name  = 'grad_orient';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing orientation using gradient');
            case 1
                opt_name  = 'grad_plane';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing tree plane distance');
            otherwise
                opt_name  = 'orient';
                constraints = 'no';
                k0 = 1;
                disp('Optimizing orientation with task object');
                
        end
        
        for j = 1 : (N-1)
            % Null space optimization (IK)
            if use_ik_opt
                J = robot.jacob0(pick{i}.ik.opt{k}.q(j,:));
                Jpinv = J' * ((J * J')^-1);
                pick{i}.ik.opt{k}.q0 = k0 * null_opt(robot, ...
                    opt_name, pick{i}.ik.opt{k}.q(j,:), constraints);
                qns = (eye(8) - Jpinv * J) * pick{i}.ik.opt{k}.q0';
                pick{i}.ik.opt{k}.qdot(j,:) = Jpinv * pick{i}.ve(j,:)'+qns;
                pick{i}.ik.opt{k}.q(j+1,:) = pick{i}.ik.opt{k}.q(j,:) + ...
                    (pick{i}.ik.opt{k}.qdot(j,:) * dt);
                % Check whether the joints are within the limits or not
                % check_jlim(robot, task{i}.ik.opt{k}.q(j+1,:));
            end
            
            % Null space optimization (CLIK)
            delta_k = tr2delta(robot.fkine(pick{i}.clik.opt{k}.q(j,:)), ...
                pick{i}.TC(:,:,j));
            J = robot.jacob0(pick{i}.clik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            pick{i}.clik.opt{k}.q0 = k0 * null_opt(robot, opt_name, ...
                pick{i}.clik.opt{k}.q(j,:), constraints);
%             pick{i}.clik.opt{k}.q0 = k0 * dev_free_opt(robot, opt_name, ...
%                 pick{i}.clik.opt{k}.q(j,:));
%             pick{i}.clik.opt{k}.q0 = k0 * grad_est(robot, opt_name, ...
%                 pick{i}.clik.opt{k}.q(j,:));
            pick{i}.clik.opt{k}.q0
            qns = (eye(8) - Jpinv * J) * pick{i}.clik.opt{k}.q0';
            pick{i}.clik.opt{k}.qdot(j,:) = Jpinv * (pick{i}.ve(j,:)' + ...
                pick{i}.clik.opt{k}.K * delta_k) + qns;
            pick{i}.clik.opt{k}.q(j+1,:) = pick{i}.clik.opt{k}.q(j,:) + ...
                (pick{i}.clik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            check_jlim(robot, pick{i}.clik.opt{k}.q(j+1,:));
        end
    end
    disp('End of optimization phase');
end

%% Trajectory without cart considering joint limits (Pick)
N = 200;  
T1 = transl(fruit{1}(1), fruit{1}(2), fruit{1}(3)) * robot.base;
TCR = ctraj(T0, T1, N);

pick{1}.q_no_cart(1,:) = qn(3:8);
for i = 2: N
    pick{1}.q_no_cart(i,:) = arm.ikcon(TCR(:, :, i), ...
        pick{1}.q_no_cart(i-1,:));
end