%% Optimization using gradient

global dist_grad q1 q2 q3 q4 q5 q6 q7 q8 q_sym

syms q1 q2 q3 q4 q5 q6 q7 q8 real

q_sym = [q1 q2 q3 q4 q5 q6 q7 q8];

dist_grad = gradient((1 / (2*robot.n)) * sumsqr((q_sym' - j_mid) ./ (robot.qlim(:,2) - ...
robot.qlim(:,1))));

% manip_grad = gradient(sqrt(det(jacob0(robot,q_sym) * jacob0(robot,q_sym)')));
cond_grad = gradient(cond(robot.jacob0(q_sym)));

%% Numerical gradient of manipulability

step = deg2rad(1);

delta = 15;
q1 = deg2rad(qmin(1, :));
q2 = deg2rad(qmin(2, :));
q3 = deg2rad(qmin(3, :));
q4 = deg2rad(qmin(4, :));
q5 = deg2rad(qmin(5, :));
q6 = deg2rad(qmin(6, :));

q1max = deg2rad(qmax(1, :));
q2max = deg2rad(qmax(2, :));
q3max = deg2rad(qmax(3, :));
q4max = deg2rad(qmax(4, :));
q5max = deg2rad(qmax(5, :));
q6max = deg2rad(qmax(6, :));

while end_mcm == 0;
    q = [q1 q2 q3 q4 q5 q6]
    work_manipl(i, :) = robot.maniplty(q);
    i = i + 1;
    q1 = q1 + deg2rad(delta);
    if (q1 > q1max)
        q1 = deg2rad(qmin(1, :));
        q2 = q2 + deg2rad(delta);
        if (q2 > q2max)
            q2 = deg2rad(qmin(2, :));
            q3 = q3 + deg2rad(delta);
            if (q3 > q3max)
                q3 = deg2rad(qmin(3, :));
                q4 = q4 + deg2rad(delta);
                if (q4 > q4max)
                    q4 = deg2rad(qmin(4, :));
                    q5 = q5 + deg2rad(delta);
                    if (q5 > q5max)
                        q5 = deg2rad(qmin(5, :));
                        q6 = q6 + deg2rad(delta);
                        if (q6 > q6max)
                            break;
                        else
                            continue;
                        end
                    else
                        continue;
                    end
                else
                    continue;
                end
            else
                continue;
            end
        else
            continue;
        end
    else
        continue;
    end

end