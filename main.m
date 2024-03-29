clear;
tic;

max_generation = 200;
load_flag = 1;
save_flag = 1;

average_number_non_disabled_connections = [];
average_number_hidden_nodes = [];
max_overall_fitness = [];
%Init population
population_size = 200;%150;
input_nodes = 3;
output_nodes = 1;
vector_connected_input_nodes = [output_nodes, input_nodes];
%for a fully connected input layer
%vector_connected_input_nodes = 1:input_nodes;
%% Speciation parameters
%Structure with various information on single species
%Data will be used for fitness sharing/reproduction/visualization
species_record(1).ID = 0;
species_record(1).number_individuals = 0;
%matrix shape (4, columns): number of generation; mean raw fitness; max
%raw; index of individual in population with max fitness
%fitness; 
species_record(1).generation_record = [];

speciation.c1 = 1.0;
speciation.c2 = 1.0;
speciation.c3 = 0.4;
speciation.threshold = 3;
%% Reproduction parameters
%if you have a fitness function which has a large spread, you might want to increase this threshold 
stagnation.threshold = 1e-2;
%if max fitness of species has stayed within stagnation.threshold in the last stagnation.number_generation generations, all its fitnesses will be reduced to 0, so it will die out
%Computation is done the following way: the absolute difference between the average max fitness of the last stagnation.number_generation generations and the max fitness of each of these generations is computed and compared to stagnation.threshold. 
%if it stays within this threshold for the indicated number of generations, the species is eliminated 
stagnation.number_generation = 15;
refocus.threshold = 1e-2;
refocus.number_generation = 20;

%initial setup
initial.kill_percentage = 0.2;
%kill_percentage used in species which have more individuals than number for kill
initial.number_for_kill = 5;
initial.number_copy = 5;

selection.pressure = 2; %number between 1.1 and 2.0 determines selective pressure towards most fit individual of species
%% Crossover
%percentage govers the way in which new population will be composed from old population. 
crossover.percentage = 0.8;%species with one individual can only perform mutation
crossover.probability_interspecies = 0.0;
crossover.probability_multipoint = 0.6;
%% Mutation
mutation.probability_add_node = 0.03;
mutation.probability_add_connection = 0.05;%0.05;
mutation.probability_recurrency = 0.1;%0.0;
mutation.probability_mutate_weight = 0.1;%0.9;
mutation.weight_cap = 8; %Weights will be restricted from -mutation.weight_cap to mutation.weight_cap
mutation.weight_range = 5; %rand distribution with width mutation.weight_range, centered on 0. range 5 -> [-2.5, 2.5] distr
mutation.probability_gene_reenabled = 0.25; % Probability of a connection gene being reenabled in offspring if it was inherited disabled

%% Main Algorithm

if load_flag == 0
    %call function to init population
    [population, innovation_record] = initial_population(population_size, input_nodes, output_nodes, vector_connected_input_nodes);
    %initial speciation
    number_connections = (length(vector_connected_input_nodes) + 1) * output_nodes;
    %put first individual in species one and update species_record
    population(1).species = 1;
    matrix_reference_individuals = population(1).connectiongenes(4, :);%species reference matrix (abbreviated, only weights, since their is no topology difference in initial population)
    species_record(1).ID = 1;
    species_record(1).number_individuals = 1;
    
    %Loop through rest of individuals and either assign to existing species
    %or create new species and use first individual of new species as
    %reference
    for index_individual = 2 : size(population, 2)
        assigned_existing_species_flag = 0;
        new_species_flag = 0;
        index_species = 1;
        %Loops through existing species, terminates when either the individual 
        %is assigned to existing species or there are no more species to test 
        %it against, which means its a new species
        while assigned_existing_species_flag == 0 & new_species_flag == 0
            %Compatibility distance, abbreviated, only average distance considered
            distance = speciation.c3 * sum(abs(population(index_individual).connectiongenes(4, :) - matrix_reference_individuals(index_species, :))) / number_connections;
            if distance < speciation.threshold % If within threshold -> assign to the existing species
                population(index_individual).species = index_species;
                assigned_existing_species_flag = 1;
                %incrementation
                species_record(index_species).number_individuals = species_record(index_species).number_individuals + 1;
            end
            %incrementation
            index_species = index_species + 1;
            if index_species > size(matrix_reference_individuals, 1) & assigned_existing_species_flag == 0
                new_species_flag = 1;
            end
        end
        %Check for new species, if it is, update the species_record and use
        %individual as reference for new species
        if new_species_flag == 1
            population(index_individual).species = index_species;
            matrix_reference_individuals = [matrix_reference_individuals;
                                            population(index_individual).connectiongenes(4, :)];
            species_record(index_species).ID = index_species;
            %if number individuals in species is zero, that species is extinct
            species_record(index_species).number_individuals = 1;
        end
    end
    generation = 1;
else % Start with saved version of evolution
    load 'neatsave'
end

%% Generational loop
% [theta, theta_dot, error] -> scatterplot
memoize_best_individuals = [];
flag_solution = 0;
while generation < max_generation & flag_solution == 0
    if save_flag == 1 %Backup copies of current generation
        save 'neatsave' population generation innovation_record species_record
    end
    %call evaluation function (XOR), fitnesses of individuals will be stored in population(:).fitness
    %IMPORTANT: reproduction assumes an (all positive!) evaluation function
    %where a higher value means better fitness 
    %(in other words, the algorithm is geared towards maximizing a 
    %fitness function which can only assume values between 0 and +Inf
    
    %population with updated fitnesses
    %population = xor_experiment_vec(population, input_nodes);
    
    %% Equations of motion and Pendulum
    % Each individuals are evaluated on a set of random init_conditions
    
    l = 1;
    m = 1;
    g = 9.81;%9.81;
    damping = 1.5;
    max_torque = 20;

    tsim = 4;
    dt = 0.1;
    
    number_of_samples = 100;%55;
    population_size = size(population, 2);
    
    theta_ddot = zeros(number_of_samples, tsim/dt + 1, population_size);
    theta_dot = zeros(number_of_samples, tsim/dt + 1 + 1, population_size);
    theta = zeros(number_of_samples, tsim/dt + 1 + 1, population_size);
    torque_list = zeros(number_of_samples, tsim/dt + 1 + 1, population_size);
    t_list = 0:dt:tsim;
    
    %% Distribution initializer
    %We feed initial conditions -> inputs shape: (number_of_samples, inputs, population_size)
    %inputs = ones(number_of_samples, input_nodes, population_size);
    %max_distr = 2 * pi;
    %distribution = (-1 + (1+1) * rand(number_of_samples, 3, 1)) * max_distr;%2 * pi;
    %distribution(:, 1, 1) = ones(number_of_samples, 1, 1) .* 0; %initial position -> 0
    %distribution(:, 2, 1) = ones(number_of_samples, 1, 1) .* 0; %initial velocity -> 0
    %distribution(:, 3, 1) = ones(number_of_samples, 1, 1) .* pi/2; %desired position
    %for individual = 1 : population_size
        %inputs dim: (number_of_samples, 3, population_size) -> same set of rand numbers for each individual (equal chances to succeed) 
    %    inputs(:, :, individual) = inputs(:, :, individual) .* distribution; 
    %end
    %% Distribution initializer (update) distributions are independant
    inputs = getRandInitDist(input_nodes, population_size, number_of_samples, 0.10);
    
    
    %memoize_score = [];
    are_targets_locked_individual = zeros(1, size(population, 2));
    theta(:, 1, :) = inputs(:, 1, :);
    theta_dot(:, 1, :) = inputs(:, 2, :);
    counter = length(t_list);% * population_size;
    for t = 1:length(t_list)
        time_array = [t, dt, tsim];
        %Output for one individual in population
        %[population, outputs, memoize_score, are_targets_locked_individual] = neuroevolutive_pendulum(population, inputs, memoize_score, time_array, are_targets_locked_individual, 1);
        [population, outputs, are_targets_locked_individual] = neuroevolutive_pendulum(population, inputs, time_array, are_targets_locked_individual, 0.98);
        torque_list(:, t, :) = outputs * max_torque;
        theta_ddot(:, t, :) = equation_of_motion(theta(:, t, :), theta_dot(:, t, :), torque_list(:, t, :), m, g, l, damping);
        theta_dot(:, t+1, :) = theta_ddot(:, t, :) * dt + theta_dot(:, t, :);
        theta(:, t+1, :) = theta_dot(:, t+1, :) * dt + theta(:, t, :);
        %Assigning new velocity and position to inputs vector
        inputs = [theta(:, t+1, :), theta_dot(:, t+1, :), inputs(:, 3, :)];

        counter = counter - 1
        locked_individuals = sum(are_targets_locked_individual)
    end
    %% Plot scatter scores dim(theta(:, 1), theta_dot(:, 2), error(:, 3))
    %figure(5)
    %scatter3(memoize_score(:, 1)', memoize_score(:, 2)', memoize_score(:, 3)', [5], memoize_score(:, 3)');
    %colormap(hot)
    %view([0, 0, 90]) %view([30, 60, 45])
    %xlabel('theta');
    %ylabel('theta_{dot}');
    %zlabel('kernel score distribution');
    %ylim_min = -2*pi;
    %ylim_max = 2*pi;
    %set(gca,'XLim',[-2*pi 2*pi],'YLim',[ylim_min ylim_max])
    %set(gca,'XTick', -2*pi:pi:2*pi); 
    %set(gca,'XTickLabel',{'-2*pi', '-pi', '0', 'pi', '2*pi'});
    %drawnow;
        
    generation
    %% compute mean and max raw fitnesses in each species and store in species_record.generation_record
    max_fitnesses_current_generation = zeros(1, size(species_record, 2));
    
    for index_species = 1: size(species_record, 2)
        if species_record(index_species).number_individuals > 0
            [max_fitness, index_individual_max] = max(([population(:).species] == index_species) .* [population(:).fitness]);
            mean_fitness = sum(([population(:).species] == index_species) .* [population(:).fitness]) / species_record(index_species).number_individuals;
            %Compute stagnation vector (last stagnation.number_generation - 1 max fitnesses plus current fitness
            if size(species_record(index_species).generation_record, 2) > stagnation.number_generation - 2
                stagnation_vector = [species_record(index_species).generation_record(3, size(species_record(index_species).generation_record, 2) - stagnation.number_generation + 2 : size(species_record(index_species).generation_record, 2)), max_fitness];
                if sum(abs(stagnation_vector - mean(stagnation_vector)) < stagnation.threshold) == stagnation.number_generation %Check for stagnation
                    mean_fitness = 0.01; %Set mean fitness to small value to eliminate species (cannot be set to 0, if only one species is present, we would have divided by zero in fitness sharing. Anyways, with onmy one species present, we have to keep it)
                end
            end
            species_record(index_species).generation_record = [species_record(index_species).generation_record, [generation;
                                                                                                                 mean_fitness;
                                                                                                                 max_fitness;
                                                                                                                 index_individual_max]];
            max_fitnesses_current_generation(1, index_species) = max_fitness;
        end
    end
    %% Check for refocus
    [top_fitness, index_top_species] = max(max_fitnesses_current_generation);
    if size(species_record(index_top_species).generation_record, 2) > refocus.number_generation
        index1 = size(species_record(index_top_species).generation_record, 2) - refocus.number_generation;
        index2 = size(species_record(index_top_species).generation_record, 2);
        if sum(abs(species_record(index_top_species).generation_record(3, index1:index2) - mean(species_record(index_top_species).generation_record(3, index1:index2))) < refocus.threshold) == refocus.number_generation
            [discard, vector_cull] = sort(-max_fitnesses_current_generation);
            vector_cull = vector_cull(1, 3:sum(max_fitnesses_current_generation > 0));
            for index_species = 1 : size(vector_cull, 2)
                index_cull = vector_cull(1, index_species);
                species_record(index_cull).generation_record(2, size(species_record(index_cull).generation_record, 2)) = 0.01;
            end
        end
    end
    
    %% Visualisation fitness & species
    a = 0;
    b = 0;
    for index_individual = 1 : size(population, 2)
        a = a + sum(population(index_individual).connectiongenes(5, :) == 1);
        b = b + sum(population(index_individual).nodegenes(2, :) == 3);
    end
    average_number_non_disabled_connections = [average_number_non_disabled_connections, [a / population_size;
                                                                                        generation]];
    average_number_hidden_nodes = [average_number_hidden_nodes, [b / population_size;
                                                                 generation]];
    c = [];
    for index_species = 1 : size(species_record, 2)
        c = [c, species_record(index_species).generation_record(1:3, size(species_record(index_species).generation_record, 2))];
    end
    max_overall_fitness = [max_overall_fitness, [max(c(3,:).*(c(1,:) == generation));
                                                 generation]];
    
    maximal_fitness = max(c(3,:).*(c(1,:)==generation))
    %if maximal_fitness > 0.8 
    %  flag_solution = 1;
    %end
    figure(3);
    subplot(2, 2, 1);
    plot(average_number_non_disabled_connections(2,:),average_number_non_disabled_connections(1,:));
    ylabel('non disabled con');
    subplot(2,2,2);
    plot(average_number_hidden_nodes(2,:),average_number_hidden_nodes(1,:));
    ylabel('num hidden nodes');
    subplot(2,2,3);
    plot(max_overall_fitness(2,:),max_overall_fitness(1,:));
    ylabel('max fitness');
    drawnow;
    %% Attraction basin
    k = g;
    Z = k*theta.^2 / 2 + m * theta_dot.^2 / 2;
    Z_scaled = Z ;%/ sum(sum(Z));
    %% E_derivative
    energy_der = - damping * theta_dot.^2;
    
    %% Energy + energy derivative
    animation = 0;
    %% TODO when necessary
    if (animation == 1)
        for individual = 1 : size(population, 2)
            if population(individual).fitness == maximal_fitness %>= 0.8 * maximal_fitness
            end
        end
    else
        %% Show every individual
        for individual = 1 : size(population, 2)
            if population(individual).fitness <= maximal_fitness %>= 0.8 * maximal_fitness
                %random_rgb = rand(1, 3);

                figure(1);
                title('Bassin of attraction');
                plot(transpose(theta(:, 1:t, individual)), transpose(theta_dot(:, 1:t, individual)));%, 'color', random_rgb)%, transpose(Z_scaled(:, 1:t)));%, 'Color', 'red');
                ylim_min = -3*pi;
                ylim_max = 3*pi;
                set(gca,'XLim',[-3*pi 3*pi],'YLim',[ylim_min ylim_max],'ZLim',[0 1000])
                set(gca,'XTick', -4*pi:pi:4*pi); 
                set(gca,'XTickLabel',{'-4*pi', '-3*pi', '-2*pi', '-pi', '0', 'pi', '2*pi', '3*pi', '4*pi'});
                xlabel('theta');
                ylabel('thetadot');
                %zlabel('k/2 * theta² + m/2 * thetadot²');
                hold on
                x = inputs(:, 3, individual);
                low_bound = ylim_min;
                upper_bound = ylim_max;
                ystart = ones(length(x), 1) .* low_bound;
                ystop = ones(length(x), 1) .* upper_bound;
                plot([x.'; x.'], [ystart.'; ystop.'], '--');
                hold off
                drawnow;
                

                figure(2);
                title('Energy derivative');
                plot(transpose(energy_der(:, 1:t, individual)));%, 'color', random_rgb);

                drawnow;
                hold all
            end
            hold off
        end
        %% Show best individual
        for individual = 1 : size(population, 2)
            if population(individual).fitness == maximal_fitness 
                %random_rgb = rand(1, 3);
                
                figure(1);
                title('Bassin of attraction');
                plot(transpose(theta(:, 1:t, individual)), transpose(theta_dot(:, 1:t, individual)));%, 'color', random_rgb)%, transpose(Z_scaled(:, 1:t)));%, 'Color', 'red');
                ylim_min = -3*pi;
                ylim_max = 3*pi;
                set(gca,'XLim',[-3*pi 3*pi],'YLim',[ylim_min ylim_max],'ZLim',[0 1000])
                set(gca,'XTick', -4*pi:pi:4*pi); 
                set(gca,'XTickLabel',{'-4*pi', '-3*pi', '-2*pi', '-pi', '0', 'pi', '2*pi', '3*pi', '4*pi'});
                xlabel('theta');
                ylabel('thetadot');
                %zlabel('k/2 * theta² + m/2 * thetadot²');
                hold on
                x = inputs(:, 3, individual);
                low_bound = ylim_min;
                upper_bound = ylim_max;
                ystart = ones(length(x), 1) .* low_bound;
                ystop = ones(length(x), 1) .* upper_bound;
                plot([x.'; x.'], [ystart.'; ystop.'], '--');
                hold off
                drawnow;
                

                figure(2);
                title('Energy derivative');
                plot(transpose(energy_der(:, 1:t, individual)));%, 'color', random_rgb);

                drawnow;
                hold all
            end
            hold off
        end
    end
    %% Store best individual
    for individual = 1 : size(population, 2)
            if population(individual).fitness == maximal_fitness
                memoize_best_individuals = [memoize_best_individuals, individual];
            end
    end
    %% Simulation
    %while false
    if mod(generation, 1) == 0 || generation == 1
        for ind = 1:size(population, 2)
            %if population(ind).fitness >= 0.9%0.8
            %show_me_array = max_overall_fitness(2,:)
            %if population(ind).fitness >= 0.8 * max(max_overall_fitness(2,1:length(max_overall_fitness)-1))
            for sample = 1 : number_of_samples
            if population(ind).fitness == maximal_fitness %>= 0.8 * maximal_fitness
            for i = 1:length(t_list)
                %% Plotting
                figure(4);
                subplot(4, 2, 1);
                plot(t_list(1:i), theta_ddot(sample, 1:i, ind));
                ylabel('theta ddot');
                subplot(4, 2, 2);
                plot(t_list(1:i), theta_dot(sample, 1:i, ind));
                ylabel('theta dot');
                subplot(4, 2, 3);
                plot(t_list(1:i), theta(sample, 1:i, ind));
                hold on
                plot(t_list(1:i), ones(1, size(t_list(1:i), 2)) .* inputs(sample, 3, ind)); %plotting desired_position
                ylabel('theta');
                hold off
                subplot(4, 2, 4);
                plot(theta(sample, 1:i, ind), theta_dot(sample, 1:i, ind));
                ylabel('phase');

                %Pendulum
                subplot(4, 2, [5 8]);
                title(['generation ',num2str(generation), '/', num2str(max_generation), ' | pendulum number', num2str(ind), '/', num2str(population_size), ' | sample number', num2str(sample), '/', num2str(number_of_samples), ' | fitness = ', num2str(population(ind).fitness)]);
                orig = [0 0];
                axis(gca, 'equal'); %Aspect ratio of the plot
                axis([-1.2, 1.2, -1.2, 1.2]);%limits of the plot
                grid on;
                %Loop for animation
                %tic;
                %Mass point
                P = l * [sin(theta(sample, i, ind)) -cos(theta(sample, i, ind))];
                %Circle in origin
                orig_circle = viscircles(orig, 0.01);
                %Pendulum
                pend = line([orig(1) P(1)], [orig(2) P(2)], 'Color', 'black' );
                %Ball
                ball = viscircles(P, 0.05);
                %Thrust vector + rescale
                F = torque_list(sample, i, ind) / abs(sum(torque_list(sample, :, ind))) * 10;
                torque_list(1, i, ind)
                x_vec = P(1) + (-1) * F * cos(theta(sample, i, ind) + pi);
                y_vec = P(2) + (-1) * F * sin(theta(sample, i, ind) + pi);
                %vector = line([P(2), x_vec], [P(1), y_vec])
                vector = line([x_vec, P(1)], [y_vec, P(2)], 'Color', 'red')
                %time interval to update the plot (-> drawnow)
                %pause(0.01);
                drawnow;
                %Delete previous objects
                if i <= length(t_list)
                    delete(pend);
                    delete(ball);
                    delete(orig_circle);
                    delete(vector);
                end
                %toc;
            continue
            end
            end %end of if
            end %end of samples
        end
    end
    %%
    
    if flag_solution == 0
        %call reproduction function with parameters, current population and
        %species record, returns new population, new species record and new
        %innovation record
        [population, species_record, innovation_record] = reproduce(population, species_record, innovation_record, initial, selection, crossover, mutation, speciation, generation, population_size);
        toc;
    end
    generation = generation + 1;
end