%% Place Phase

% Tasks definition
place = cell(1, 3);

% Place phase for each task
place{1} = struct;
place{2} = struct;
place{3} = struct;

% Basket position
basket = T1(1:3,4);

%% Cartesian trajectories and inverse differential kinematics

N = 200;
dt = 0.01;
T1 = robot.fkine(qn);

for i = 1 : length(place)
    disp(['************ TASK ' num2str(i) ' - Place Phase ************']);
    disp('Computing Cartesian trajectory...');
    T0 = transl(fruit{i}(1), fruit{i}(2), fruit{i}(3)) * robot.base;
    
    place{i}.TC = ctraj(T0, T1, N);
    place{i}.ve = zeros(N-1, 6);
    
    % Inverse differential kinematics struct (IK)
    place{i}.ik = struct;
    % Closed-loop inverse kinematics struct (CLIK)
    place{i}.clik = struct;
    % Optimization structs
    place{i}.ik.opt = cell(3,1);
    place{i}.clik.opt = cell(3,1);
    
    place{i}.ik.no_opt.q = zeros(N, 8);
    place{i}.ik.no_opt.q(1,:) = pick{i}.ik.no_opt.q(end,:);
    place{i}.ik.no_opt.qdot = zeros(N-1, 8);
    
    place{i}.clik.no_opt.q = zeros(N, 8);
    place{i}.clik.no_opt.q(1,:) = pick{i}.clik.no_opt.q(end,:);
    place{i}.clik.no_opt.qdot = zeros(N-1, 8);
    place{i}.clik.no_opt.K = eye(6);  % Closed-loop gain matrix
    
    
    disp(['Computing differential IK and CLIK without null space'...
        ' optimizations']);
    for j = 1 : (N-1)
        place{i}.ve(j,:) = tr2delta(place{i}.TC(:,:,j), ...
            place{i}.TC(:,:,j+1)) / dt;
        
        % No null space optimization inverse differential kinematics (IK)
        J = robot.jacob0(place{i}.ik.no_opt.q(j,:));
        Jpinv = J' * ((J * J')^-1);
        place{i}.ik.no_opt.qdot(j,:) = Jpinv * place{i}.ve(j,:)';
        place{i}.ik.no_opt.q(j+1,:) = place{i}.ik.no_opt.q(j,:) + ...
            (place{i}.ik.no_opt.qdot(j,:) * dt);

        % Closed-loop inverse kinematics (CLIK)
        delta_k = tr2delta(robot.fkine(place{i}.clik.no_opt.q(j,:)), ...
            place{i}.TC(:,:,j));
        J = robot.jacob0(place{i}.clik.no_opt.q(j,:));
        Jpinv = J'*((J * J')^-1);
        place{i}.clik.no_opt.qdot(j,:) = Jpinv * (place{i}.ve(j,:)' + ...
            place{i}.clik.no_opt.K * delta_k);
        place{i}.clik.no_opt.q(j+1,:) = place{i}.clik.no_opt.q(j,:) + ...
            (place{i}.clik.no_opt.qdot(j,:) * dt);
    end
end

%% Differential IK and CLIK with null space optimizations

for i = 1 : length(place)
    disp(['************ TASK ' num2str(i) ' - Place Phase ************']);
    disp('Null Space Optimizations');
    
    % The number of optimizations to be performed is expressed by the
    % number 'k' of loop iterations. The specific optimizations are
    % specified inside the switch block and the parameters can be checked
    % in the 'null_opt.m' file.
    for k = 1 : 2
        place{i}.clik.opt{k} = struct;
        place{i}.clik.opt{k}.q = zeros(N,8);
        place{i}.clik.opt{k}.q(1,:) = pick{i}.clik.no_opt.q(end,:);
        place{i}.clik.opt{k}.qdot = zeros(N-1,8);
        place{i}.clik.opt{k}.q0 = zeros(1,8);
        place{i}.clik.opt{k}.K = eye(6);  % Closed-loop gain matrix
        qns = zeros(1,8);
        
        switch k
            case 1
                opt_name = 'dist';
                options = {};
                k0 = 1;
                disp(['Optimizing distance from mechanical joint ' ...
                    'limits using fminunc']);
            case 2
                opt_name = 'dist';
                options = {'gradient_est'};
                k0 = 1;
                disp(['Optimizing distance from mechanical joint ' ...
                    'limits using gradient estimation']);
            otherwise
                error('Invalid optimization index');
                
        end
        
        for j = 1 : (N-1)       
            % Null space optimization (CLIK)
            delta_k = tr2delta(robot.fkine(place{i}.clik.opt{k}.q(j,:)), ...
                place{i}.TC(:,:,j));
            J = robot.jacob0(place{i}.clik.opt{k}.q(j,:));
            Jpinv = J' * ((J * J')^-1);
            place{i}.clik.opt{k}.q0 = k0 * null_opt(robot, opt_name, ...
                place{i}.clik.opt{k}.q(j,:), options);
            qns = (eye(8) - Jpinv * J) * place{i}.clik.opt{k}.q0';
            place{i}.clik.opt{k}.qdot(j,:) = Jpinv * (place{i}.ve(j,:)' + ...
                place{i}.clik.opt{k}.K * delta_k) + qns;
            place{i}.clik.opt{k}.q(j+1,:) = place{i}.clik.opt{k}.q(j,:) + ...
                (place{i}.clik.opt{k}.qdot(j,:) * dt);
            % Check whether the joints are within the limits or not
            % check_jlim(robot, place{i}.clik.opt{k}.q(j+1,:));
        end
    end
    disp('End of optimization phase');
end

%% Trajectory without cart considering joint limits (Place)
N = 200;  

T0 = transl(fruit{1}(1), fruit{1}(2), fruit{1}(3)) * robot.base;
T1 = robot.fkine(qn);
TCR = ctraj(T0, T1, N);

place{1}.q_no_cart(1,:) = qn(3:8);
for i = 2: N
    place{1}.q_no_cart(i,:) = arm.ikcon(TCR(:, :, i), ...
        place{1}.q_no_cart(i-1,:));
end