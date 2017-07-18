function [ grad ] = grad_est( obj_f, q_k )
%grad_est Numerical gradient estimation usign pseudo-derivative free method

global D;

% Step size. This value can be changed in order to improve the gradient
% estimation.
tk = 1;

numBasis = length(D);
max = obj_f(q_k);
i_max = -1;

for i = 1 : numBasis
    if (obj_f(q_k + tk * D(i,:)) > max)
        max = obj_f(q_k + tk * D(i,:));
        i_max = i;
    end;
end;

if (i_max == -1)
    % q_k is a local maximum
    grad = zeros(1, 8);
else
    grad = D(i_max,:);
end

end

