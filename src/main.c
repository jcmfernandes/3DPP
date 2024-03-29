#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include "algo.h"
#include "bitmap.h"
#include "config.h"
#include "heightmap.h"
#include "log.h"
#include "grids.h"

typedef enum { GS, CGS, PGS, J, PJ } ALGO;

potential_grid_cell_t static potential_grid_cells_1[WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z];
potential_grid_cell_t static potential_grid_cells_2[WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z];
obstacles_grid_cell_t static obstacles_grid_cells[obstacles_grid_required_cells(WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z)];

int main(int argc, char * argv[])
{
	potential_grid_t potential_grid1, potential_grid2;
	obstacles_grid_t obstacles_grid;
	position_t starting_position, goal_position;

	potential_grid1 = (potential_grid_t) potential_grid_cells_1;
	potential_grid2 = (potential_grid_t) potential_grid_cells_2;
	obstacles_grid_create(&obstacles_grid, (obstacles_grid_cell_t *) &obstacles_grid_cells, sizeof(obstacles_grid_cells));

	starting_position[0] = STARTING_POINT_X;
	starting_position[1] = STARTING_POINT_Y;
	starting_position[2] = heightmap[STARTING_POINT_X][STARTING_POINT_Y] + 2;

	goal_position[0] = GOAL_X;
	goal_position[1] = GOAL_Y;
	goal_position[2] = heightmap[GOAL_X][GOAL_Y] + 2;

	logStart("initialize obstacle map");
	// Recode heightmap to global_obstacles
	for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
		for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
			uint64_t height = heightmap[a][b];

			// some regions cannot be flown over
			if (height > HEIGHT_LIMIT) height = WORLD_SIZE_Z;

			for (uint64_t c = 0; c < height; c++) {
				bitmap_set(&obstacles_grid, GRID_INDEX(a, b, c));
			}
		}
	}
	logEnd("initialize obstacle map");

	logStart("initialize weightmap from obstacle map");
	for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
		for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
			for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
				if (position_is_goal(goal_position, a, b, c)) {
					potential_grid1[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
				} else if (position_is_obstacle(obstacles_grid, a, b, c)) {
					potential_grid1[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
				} else {
					potential_grid1[GRID_INDEX(a, b, c)] = WEIGHT_INIT;
				}
			}
		}
	}
	logEnd("initialize weightmap from obstacle map");

	logStart("process the potential grid");
	switch (ALGORITHM) {
	case GS:
		calc_potential_gs(potential_grid1, obstacles_grid, goal_position, GS_ITERATIONS);
		break;
	case CGS:
		calc_potential_gs_conv(potential_grid1, obstacles_grid, goal_position, CGS_CONVERGENCE_DELTA);
		break;
	case PGS:
		calc_potential_pgs(potential_grid1, obstacles_grid, goal_position, PGS_ITERATIONS, PGS_X_COMPARTMENTS, PGS_Y_COMPARTMENTS, PGS_Z_COMPARTMENTS);
		break;
	case J:
		calc_potential_j(potential_grid1, potential_grid2,
				obstacles_grid, goal_position, J_ITERATIONS);
		break;
	case PJ:
		calc_potential_pj(potential_grid1, potential_grid2,
				obstacles_grid, goal_position, PJ_ITERATIONS, PJ_THREADS);
		break;
	default:
		perror("error: inexistent algorithm.\n");
		break;
	}
	logEnd("process the potential grid");

	logStart("findWaypoints");
	find_waypoints(potential_grid1, obstacles_grid, starting_position, goal_position, stdout);
	logEnd("findWaypoints");

	return 0;
}
