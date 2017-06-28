i = 1;
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

end_mcm = 0;

while end_mcm == 0;
    q = [q1 q2 q3 q4 q5 q6]
    scatter(i, :) = transl(arm.fkine(q));
    work_manipl(i, :) = arm.maniplty(q);
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

