function [ fig ] = qplot_8dof( q )
%QPLOT_8DOF Plots the q values for a 8 DoF robot
%   Takes as input a series of joint values and plots them

fig = figure;

hold on
t = (1:size(q,1));
plot(t, q(:,1:3))
plot(t, q(:,4:6), '--')
plot(t, q(:,7:8), ':'); 
grid on
xlabel('Time Steps')
ylabel('q')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8');
hold off
xlim([t(1), t(end)]);

end

