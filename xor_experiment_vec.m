function population_with_new_fitnesses = xor_experiment_vec(population, input_nodes);

population_with_new_fitnesses = population;
%Threshold to judge whether state of node has changed significantly since
%last iteration or not
no_change_threshold = 1e-3;
number_individuals = size(population, 2);

input_pattern = [[0 0];
                 [0 1];
                 [1 0];
                 [1 1]];
output_pattern = [[0];
                  [1];
                  [1];
                  [0]];

outputs = zeros(number_individuals)
for index_individual = 1 : number_individuals
    number_nodes = size(population(index_individual).nodegenes, 2);
    number_connections = size(population(index_individual).connectiongenes, 2);
    individual_fitness = 0;
    output = [];
    %following code assumes node: 1-2 -> inputs
    %                             3   -> bias
    %                             4   -> output
    %rest arbitrary (if existent, will be hidden nodes)
    %set node output states for first timestep (depending on input states)
    for index_pattern = 1 : size(input_pattern, 1) % number of rows -> 4 for XOR
        %bias (4, 3) for XOR (4, 4) for robots stability
        population(index_individual).nodegenes(4, input_nodes+1) = 1; %Bias node output state set to 1
        %1:2 for XOR 1:3 for robots stability: pos vel acc
        population(index_individual).nodegenes(4, 1:input_nodes) = input_pattern(index_pattern, :); %node output states of the two input nodes are consecutively set to the XOR input pattern
        population(index_individual).nodegenes(4, input_nodes+2:number_nodes) = 0.5;
        
        no_change_count = 0;
        index_loop = 0;
        while (no_change_count < number_nodes) & (index_loop < 3 * number_connections)
            index_loop = index_loop + 1;
            vector_node_state = population(index_individual).nodegenes(4, :);
            %% Vectorization
            %begin vectorization of index_connection_loop
            %first remodel connection gene from node ID's to node index in node gene
            max_node_ID = max(population(index_individual).nodegenes(1, :));
            max_node_num = size(population(index_individual).nodegenes, 2);
            vector_remodel = zeros(1, max_node_ID);
            %population(index_individual).nodegenes(1, :) is an array of
            %indices that specifie where to attribute 1:max_node_num in
            %vector_remodel.
            vector_remodel(population(index_individual).nodegenes(1, :)) = [1:max_node_num];
            vector_nodes_from = vector_remodel(population(index_individual).connectiongenes(2, :));
            vector_nodes_to = vector_remodel(population(index_individual).connectiongenes(3, :));
            
            matrix_compute = zeros(max_node_num, number_connections);
            %actual vectorized computation (READ THIS CAREFULLY)
            matrix_compute(([1:number_connections]-1) * max_node_num + vector_nodes_to(1, :)) = population(index_individual).nodegenes(4, vector_nodes_from(:)) .* population(index_individual).connectiongenes(4, :) .* population(index_individual).connectiongenes(5, :);
            population(index_individual).nodegenes(3, :) = ones(1, number_connections) * matrix_compute';
            %end Vectorisation of index_connection_loop
            %%
            %pass on node input states to outputs for next timestep
            population(index_individual).nodegenes(4, input_nodes+2:number_nodes) = 1 ./ (1 + exp(-4.9 * population(index_individual).nodegenes(3, input_nodes+2:number_nodes)));
            %check for all nodes where the node output state has changed by less than no_change_threshold since last iteration through all the connection genes
            no_change_count = sum(abs(population(index_individual).nodegenes(4, :) - vector_node_state) < no_change_threshold);
        end
        if index_loop >= 2.7 * number_connections
            index_individual
            population(index_individual).connectiongenes;
        end
        output = [output;
                  population(index_individual).nodegenes(4, input_nodes+2)];
        individual_fitness = individual_fitness + abs(output_pattern(index_pattern, 1) - population(index_individual).nodegenes(4, input_nodes+2));
    end
    %Fitness function = (m - f_individual)^2
    population_with_new_fitnesses(index_individual).fitness = (size(input_pattern, 1) - individual_fitness)^2;
    if sum(abs(round(output) - output_pattern)) == 0
        %if we get the right output then we assign maximum fitness -> number examples^2
        population_with_new_fitnesses(index_individual).fitness = size(input_pattern, 1)^2;
    end
    output
end