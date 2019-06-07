function inputs = getRandInitDist(input_nodes, population_size, number_of_samples, percentage)
%
%
%
inputs = ones(number_of_samples, input_nodes, population_size);
number_targets = ceil(number_of_samples * percentage);

max_distr_initial_position = 2 * pi;
max_distr_initial_velocity = 3 * pi;
max_distr_desired_position = pi;

distribution_initial_position = (-1 + 2 * rand(number_of_samples, 1, 1)) * max_distr_initial_position;
distribution_initial_velocity = (-1 + 2 * rand(number_of_samples, 1, 1)) * max_distr_initial_velocity;
distribution_desired_position = (-1 + 2 * ones(number_of_samples, 1, 1));

% generate number_targets random targets
desired_targets = (-1 + 2 * rand(1, number_targets)) * max_distr_desired_position;
for target = 1 : number_of_samples
    %generate random number -> index of desired_targets that picks number into desired_target's distribution
    r = randi(number_targets);
    distribution_desired_position(target, 1, 1) = distribution_desired_position(target, 1, 1) * desired_targets(r);%* pi/2;
end
for individual = 1 : population_size
    inputs(:, 1, individual) = inputs(:, 1, individual) .* distribution_initial_position;
    inputs(:, 2, individual) = inputs(:, 2, individual) .* distribution_initial_velocity;
    inputs(:, 3, individual) = inputs(:, 3, individual) .* distribution_desired_position;
end

end