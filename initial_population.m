function [population, innovation_record] = initial_population(number_individuals, input_nodes, output_nodes, vector_connected_input_nodes);
%dim(nodegenes) = (rows, columns)
%rows = 4
%columns = input_nodes + output_odes + hidden - nodes (not existent in initial population) + 1 (bias - node)
%consecutive node ID's (upper_row)
%node type (second row): 1=input, 2=output, 3=hidden, 4=bias
%node input state, and node output state (used for evaluation, all input states zero initially, except bias node, which is always 1)

%dim(connectiongenes) = (5, number_connections)
%5 rows from top to bottom: innovation number, connection from, connection to, weight, enable bit

%innovation_record: (5, number of innovation) : innovation_number, connect_from_node, connect_to_node ... 
% the new node (if it is a new node mutation, then this node will appear in the 4th row when it is first connected. There will always be two innovations with one node mutation, since there is a connection to and from the new node. 
% In the initial population, this will be abbreviated to the Node with the highest number appearing in the last column of the record, since only this is needed as starting point for the rest of the algorithm), 
% and 5th row is generation this innovation occured (generation is assumed to be zero for the innovations in the initial population)

%compute number and matrix of initial connections (all connections btw output nodes and the nodes listed in vector_connected_input_nodes)
number_connections = (length(vector_connected_input_nodes)+1) * output_nodes;
vector_connection_from = rep([vector_connected_input_nodes, input_nodes+1], [1, output_nodes]);
vector_connection_to = [];

for index_output_node = (input_nodes+2):(input_nodes+1+output_nodes)
    vector_connection_to = [vector_connection_to, index_output_node*ones(1, length(vector_connected_input_nodes)+1)];
end
connection_matrix = [vector_connection_from;
                     vector_connection_to];

for index_individual = 1 : number_individuals
    population(index_individual).nodegenes = [1:(input_nodes+1+output_nodes);
                                              ones(1, input_nodes), 4, 2*ones(1, output_nodes);
                                              zeros(1, input_nodes), 1, zeros(1, output_nodes);
                                              zeros(1, input_nodes+1+output_nodes)];
    population(index_individual).connectiongenes = [1:number_connections; %innovation number
                                                   connection_matrix;% [connection from ; connection to]
                                                   rand(1, number_connections)*2-1; %weight
                                                   ones(1, number_connections)]; %all weights uniformly distributed in [-1, 1] all connections enabled
    population(index_individual).fitness = 0;
    population(index_individual).species = 0;
end
innovation_record = [population(index_individual).connectiongenes(1:3, :); %[innovation number; connection from; connection to]
                     zeros(size(population(index_individual).connectiongenes(1:2, :)))];% [new node; generation inovation occured]
innovation_record(4, size(innovation_record, 2)) = max(population(1).nodegenes(1, :)); %highest node ID for initial population