i = 1;
delta = 5;
q1 = deg2rad(qmin(1, :));
q2 = deg2rad(qmin(2, :));
q3 = deg2rad(qmin(3, :));

q1max = deg2rad(qmax(1, :));
q2max = deg2rad(qmax(2, :));
q3max = deg2rad(qmax(3, :));

end_mcm = 0;

while end_mcm == 0;
    q = [q1 q2 q3 0 0 0]
    scatter(i, :) = transl(arm.fkine(q));
    i = i + 1;
    q1 = q1 + deg2rad(delta);
    if (q1 > q1max)
        q1 = deg2rad(qmin(1, :));
        q2 = q2 + deg2rad(delta);
        if (q2 > q2max)
            q2 = deg2rad(qmin(2, :));
            q3 = q3 + deg2rad(delta);
            if (q3 > q3max)
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

end

