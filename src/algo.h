#ifndef ALGO_H_
#define ALGO_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "grids.h"

int calc_potential_gs(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t goal_position, uint32_t iterations);
int calc_potential_gs_conv(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t goal_position, double convergence);
int calc_potential_pgs(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t goal_position,uint32_t iterations, uint32_t x_compartments,
		uint32_t y_compartments, uint32_t z_compartments);
int calc_potential_j(potential_grid_t potential_grid1, potential_grid_t potential_grid2,
		obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations);
int calc_potential_pj(potential_grid_t potential_grid1, potential_grid_t potential_grid2,
		obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations, uint32_t n_threads);

int find_waypoints(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t starting_position, position_t goal_position, FILE *stream);

#endif /* ALGO_H_ */
