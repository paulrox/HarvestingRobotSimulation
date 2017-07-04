function [ q0_dot ] = deriv_free_opt( r, type, q_k )
%deriv_free_opt Optimizes the q0' values by using a derivative free method
%   It tries to find the optmial value by following the cartesian
%   coordinates directions

if strcmp(type, 'manip')
    obj_f = @(x) -r.maniplty(x, 'yoshikawa');
elseif strcmp(type, 'joint')
    % Distance from mechanical joint limits
    j_mid = mean([r.qlim(:,2) r.qlim(:,1)], 2);
    obj_f = @(x) (1 / (2*r.n)) * sumsqr((x' - j_mid) ./ (r.qlim(:,2) - ...
        r.qlim(:,1)));
end

xk = q_k;
tk = 1;
D = [eye(8);
     -eye(8)];
 
numBasis = 16;
k = 0;
beta = 0.5;
epsilon = 10e-1;
max_iter = 5;

while tk > epsilon
    i = 1;
    while i <= numBasis
        if (obj_f(xk + tk*D(i,:)) < obj_f(xk))
            xk = xk + tk*D(i,:);
            break;
        end;
        i = i+1;
    end;
    if i > numBasis
        tk = beta*tk;
    else
        if k > max_iter
            break;
        end
    end;
    
    k= k+1;
end;

q0_dot = xk;

end

