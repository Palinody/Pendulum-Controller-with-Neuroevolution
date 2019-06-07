%function [population_with_new_fitnesses, outputs, memoize_score, are_targets_locked_individual] = neuroevolutive_pendulum(population, inputs, memoize_score, time_array, are_targets_locked_individual, target_threshold)
function [population_with_new_fitnesses, outputs, are_targets_locked_individual] = neuroevolutive_pendulum(population, inputs, time_array, are_targets_locked_individual, target_threshold)
% torque -> output of each sample for each individual
%inputs -> 3d-array (num_samples, [theta_t, theta_dot_t, theta_desired], num_individuals)
%desired_position -> desired angle to reach
t = time_array(1); dt = time_array(2); tsim = time_array(3); t_list = 0:dt:tsim;
population_with_new_fitnesses = population;
%Threshold to judge whether state of node has changed significantly since last iteration or not
no_change_threshold = 1e-3;
%number_individuals = size(population, 2);

number_of_samples = size(inputs, 1);
inputs_number = size(inputs, 2);
number_individuals = size(population, 2); %size(inputs, 3);

outputs = zeros(number_of_samples, 1, number_individuals);
%memoize_score = [];
%memoize_counter = 0
for index_individual = 1 : number_individuals
    number_nodes = size(population(index_individual).nodegenes, 2);
    number_connections = size(population(index_individual).connectiongenes, 2);
    
    individual_fitness = 0;
    % if is_target_locked_individual == number_of_samples -> individual has locked every targets
    %is_target_locked_individual
    target_locked_counter = 0;
    for index_pattern = 1 : number_of_samples % number of samples -> 4 for XOR ; here we take only 1 sample at a time (stochastic)
        population(index_individual).nodegenes(4, inputs_number+1) = 1; %Bias node output state set to 1
        %1:2 for XOR 1:3 for robots stability: pos vel
        population(index_individual).nodegenes(4, 1:inputs_number) = inputs(index_pattern, :, index_individual);
        population(index_individual).nodegenes(4, inputs_number+2:number_nodes) = 0.5;
        
        no_change_count = 0;
        index_loop = 0;
        while (no_change_count < number_nodes) & (index_loop < 3 * number_connections)
            index_loop = index_loop + 1;
            vector_node_state = population(index_individual).nodegenes(4, :);
            max_node_ID = max(population(index_individual).nodegenes(1, :));
            max_node_num = size(population(index_individual).nodegenes, 2);
            vector_remodel = zeros(1, max_node_ID);
            vector_remodel(population(index_individual).nodegenes(1, :)) = [1:max_node_num];
            vector_nodes_from = vector_remodel(population(index_individual).connectiongenes(2, :));
            vector_nodes_to = vector_remodel(population(index_individual).connectiongenes(3, :));
            
            matrix_compute = zeros(max_node_num, number_connections);
            matrix_compute(([1:number_connections]-1) * max_node_num + vector_nodes_to(1, :)) = population(index_individual).nodegenes(4, vector_nodes_from(:)) .* population(index_individual).connectiongenes(4, :) .* population(index_individual).connectiongenes(5, :);
            population(index_individual).nodegenes(3, :) = ones(1, number_connections) * matrix_compute';
            %% Activation function
            %population(index_individual).nodegenes(4, inputs_number+2:number_nodes) = 1 ./ (1 + exp(-population(index_individual).nodegenes(3, inputs_number+2:number_nodes)));
            population(index_individual).nodegenes(4, inputs_number+2:number_nodes) = tanh(population(index_individual).nodegenes(3, inputs_number+2:number_nodes));
            %%
            %check for all nodes where the node output state has changed by less than no_change_threshold since last iteration through all the connection genes
            no_change_count = sum(abs(population(index_individual).nodegenes(4, :) - vector_node_state) < no_change_threshold);
        end
        if index_loop >= 2.7 * number_connections
            population(index_individual).connectiongenes;
        end
        outputs(index_pattern, 1, index_individual) = population(index_individual).nodegenes(4, inputs_number+2);
        
        %% Kernel method -> similarity(curr_vect, des_vect) = exp( - ||curr_vect - des_vect||^2 / (2*sigma^2) )
        desired_position = inputs(index_pattern, 3, index_individual);
        diff = desired_position - inputs(index_pattern, 1, index_individual);
        diff_dot = inputs(index_pattern, 2, index_individual);
        int_pos = diff * dt; %we want to minimize the total surface between desired and current angle
        %Gpos = 2; Gvel = 1/1.5; Gint = 1/1.5;
        Gpos = 1; Gvel = 1;
        % large stand_dev results in a more spread reward distribution
        sd_pos = pi / 2; % if sd_pos is too large, control will be slow but smoother (and more accurate)
        sd_vel = pi; % if sd_vel is too large, control will be fast but unstable
        % sd_int = pi / 2; //Not used
        
        % discount factor gamma
        %gamma = 0.9^t_list(t);
        % Without discount factor
        gamma = 1;
        
        % max_fitness is gonna be number_of_samples * max_similarity -> thus, max_fitness == number_of_samples
        density_kernel = gamma * exp( - (Gpos * diff)^2 / (2 * sd_pos^2) ) * exp( - (Gvel * diff_dot)^2 / (2 * sd_vel^2) );
        %% Future work: train NEAT with the following fitness function ; added integral term (approx. 18 hours)
        %density_kernel = exp( - (Gpos * diff)^2 / (2 * sd_pos^2) ) * exp( - (Gvel * diff_dot)^2 / (2 * sd_vel^2) ) * exp( - (Gint * int_pos)^2 / (2 * sd_int^2) );
        
        individual_fitness = individual_fitness + density_kernel;
        
        if density_kernel > target_threshold
            target_locked_counter = target_locked_counter + 1;
            if target_locked_counter == number_of_samples
                are_targets_locked_individual(1, index_individual) = 1;
            end
        end
        
        %% Store errors
        %if mod(index_pattern, number_of_samples/5) == 0
            %memoize_counter = memoize_counter + 1
        %    memoize_score = [memoize_score; 
        %        [inputs(index_pattern, 1, index_individual), inputs(index_pattern, 2, index_individual), density_kernel] ];
        %end
    end
    population_with_new_fitnesses(index_individual).fitness = population_with_new_fitnesses(index_individual).fitness + individual_fitness;
    % Divide by sum(0.9.^[0:dt:tsim]) wait until t == tsim because we need to wait for the scores
    if t == length(t_list)
        % With discount factor
        %population_with_new_fitnesses(index_individual).fitness = population_with_new_fitnesses(index_individual).fitness / (sum(0.9.^(t_list)) * number_of_samples);
        % Without discount factor
        population_with_new_fitnesses(index_individual).fitness = population_with_new_fitnesses(index_individual).fitness / (sum(1.^(t_list)) * number_of_samples);
    end
end
