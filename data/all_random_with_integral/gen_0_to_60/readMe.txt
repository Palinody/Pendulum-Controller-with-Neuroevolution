random: pos [-2pi, 2pi], vel [-2pi, 2pi], target [-pi, pi]
discount factor: no discount
Gpos = 1; Gvel = 1; Gint = 1;
sd_pos = pi / 2;
sd_vel = 2 * pi;
sd_int = pi / 2;
fitness = density_kernel = exp( - (Gpos * diff)^2 / (2 * sd_pos^2) ) 
			 * exp( - (Gvel * diff_dot)^2 / (2 * sd_vel^2) ) 
			 * exp( - (Gint * int_pos)^2 / (2 * sd_int^2) );
population: 200
number of samples: 100
targets: 0.3 * number_of_samples
generations: 0-60
locked_individuals: 51

memoize_best_individuals =

  Columns 1 through 20

   138   119   178     1    18     1     1     1    19    22     8   112     1     8    18     1   139    69     6    59

  Columns 21 through 40

   116   184    23    24    23   151    21   155    22    22     4     3     5     4   137    20    21    21   130    18

  Columns 41 through 60

     9    82    17    11    11    11    11     9    75    10    11    82    11    78     9    80    11    80    78     9

  Columns 61 through 70

     8     8     8    71     8     9     7     8    69     7