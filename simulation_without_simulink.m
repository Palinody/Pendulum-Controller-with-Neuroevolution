clc; clear;
l = 1;
m = 2;
g = 9.81;%9.81;
damping = 1.5;
% initial conditions
theta0 = pi/2;
thetadot0 = 2*pi; %rad/s

desired_position = pi/2
max_torque = 20; % maximum torque of the motor

tsim = 10;
dt = 0.1;%dt=0.01;

%Population size
pop_size = 5;

theta_ddot = zeros(pop_size, tsim/dt+1);
theta_dot = zeros(pop_size, tsim/dt+1 + 1);
theta = zeros(pop_size, tsim/dt+1 + 1);
torque_list = zeros(pop_size, tsim/dt+2);
t_list = 0:dt:tsim;

theta_dot(:, 1) = thetadot0;
theta(:, 1) = theta0
for ind = 1:pop_size %population size
    for i = 1:length(t_list)
       %Output for one individual in population
       torque = normrnd(0, 1) * max_torque;
       %torque = (-1 * (2)*rand()) * pi;
       torque_list(ind, i) = torque;
       theta_ddot(ind, i) = equation_of_motion(theta(ind, i), theta_dot(ind, i), torque, m, g, l, damping);
       theta_dot(ind, i+1) = theta_ddot(ind, i) * dt + theta_dot(ind, i);
       theta(ind, i+1) = theta_dot(ind, i+1) * dt + theta(ind, i);
    end
end

%% Energy
k = 1;
energy = 1/2 * [sqrt(k)*theta; sqrt(m)*theta_dot].^2;
figure(1);
plot(energy(1, :), energy(2, :));

%% Energy bassin
figure(1);
k = 1;
Z = k*theta.^2 / 2 + m * theta_dot.^2 / 2;
%for ind = 1 : pop_size
%    for t = 1 : length(t_list)
%        plot3(theta(ind, 1:t), theta_dot(ind, 1:t), Z(ind, 1:t));
%        drawnow;
%    end
%    hold on
%end
%hold off
%% Energy derivative
energy_der = - damping * theta_dot.^2;
%for ind = 1 : pop_size
%    for t = 1:length(t_list)
%        figure(2);
%        subplot(1, 1, 1);
%        plot(t_list(1:t), energy_der(ind, 1:t));
%        drawnow;
%    end
%end
%% Simulation
for ind = 1 : pop_size
    for i = 1:length(t_list)
        %% Plotting
        figure(3)
        subplot(4, 2, 1);
        plot(t_list(1:i), theta_ddot(ind, 1:i));
        ylabel('theta ddot');
        subplot(4, 2, 2);
        plot(t_list(1:i), theta_dot(ind, 1:i));
        ylabel('theta dot');
        subplot(4, 2, 3);
        plot(t_list(1:i), theta(ind, 1:i));
        hold on
        plot(t_list(1:i), ones(1, size(t_list(1:i), 2))*desired_position);
        ylabel('theta');
        hold off
        subplot(4, 2, 4);
        plot(theta(ind, 1:i), theta_dot(ind, 1:i));
        ylabel('phase');
        
        %Pendulum
        subplot(4, 2, [5 8]);
        %title(['generation ',num2str(ind)])
        orig = [0 0];
        axis(gca, 'equal'); %Aspect ratio of the plot
        axis([-1.2, 1.2, -1.2, 1.2]);%limits of the plot
        grid on;
        %Loop for animation
        %tic;
        %Mass point
        P = l * [sin(theta(ind, i)) -cos(theta(ind, i))];
        %Circle in origin
        orig_circle = viscircles(orig, 0.01);
        %Pendulum
        pend = line([orig(1) P(1)], [orig(2) P(2)]);
        %Ball
        ball = viscircles(P, 0.05);
        %time interval to update the plot
        %Thrust vector + rescale
        F = torque_list(ind, i) / length(torque_list(ind, :)) * 10
        x_vec = P(1) + (-1) * F * cos(theta(ind, i) + pi);
        y_vec = P(2) + (-1) * F * sin(theta(ind, i) + pi);
        %vector = line([P(2), x_vec], [P(1), y_vec])
        vector = line([x_vec, P(1)], [y_vec, P(2)])
        
        %pause(0.01);
        drawnow;
        %Delete previous objects if it is not the final loop
        if i <= length(t_list)
            delete(pend);
            delete(ball);
            delete(orig_circle);
            delete(vector);
        end
        %toc;

    end
end

