#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <omp.h>
#include "config.h"
#include "grids.h"

static potential_grid_cell_t calc_avg_gs(potential_grid_t potential_grid, uint64_t x, uint64_t y, uint64_t z)
{
	potential_grid_cell_t sum = 0.0;

	if (x > 0) {
		sum += potential_grid[GRID_INDEX(x - 1, y, z)];
	}
	if (x < WORLD_SIZE_X - 1) {
		sum += potential_grid[GRID_INDEX(x + 1, y, z)];
	}

	if (y > 0) {
		sum += potential_grid[GRID_INDEX(x, y - 1, z)];
	}
	if (y < WORLD_SIZE_Y - 1) {
		sum += potential_grid[GRID_INDEX(x, y + 1, z)];
	}

	if (z > 0) {
		sum += potential_grid[GRID_INDEX(x, y, z - 1)];
	}
	if (z < WORLD_SIZE_Z - 1) {
		sum += potential_grid[GRID_INDEX(x, y, z + 1)];
	}

	potential_grid_cell_t const average = sum / 6;

	return average;
}

int calc_potential_gs(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations)
{
	// Gauss-Seidel
	for (uint32_t i = 0; i < iterations; i++) {
		for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
			for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
				for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
					if (position_is_goal(goal_position, a, b, c)) {
						potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(obstacles_grid, a, b, c)) {
						potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						potential_grid[GRID_INDEX(a, b, c)] = calc_avg_gs(potential_grid, a, b, c);
					}
				}
			}
		}
	}

	return 0;
}

int calc_potential_pgs(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations, uint32_t comportaments)
{
	// Gauss-Seidel
	uint32_t i = 0;
	#pragma omp parallel shared(i)
	{
		for (; i < iterations;) {
			for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
				for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
					for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
						if (position_is_goal(goal_position, a, b, c)) {
							potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
						} else if (position_is_obstacle(obstacles_grid, a, b, c)) {
							potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
						} else {
							potential_grid[GRID_INDEX(a, b, c)] = calc_avg_gs(potential_grid, a, b, c);
						}
					}
				}
			}
		}
	}


	return 0;
}

int calc_potential_gs_conv(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid, position_t goal_position)
{
	// Gauss-Seidel
	for (uint32_t i = 0;; i++) {
		potential_grid_cell_t delta = 0.0;

		for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
			for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
				for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
					if (position_is_goal(goal_position, a, b, c)) {
						potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(obstacles_grid, a, b, c)) {
						potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						potential_grid_cell_t const initial_value = potential_grid[GRID_INDEX(a, b, c)];
						potential_grid[GRID_INDEX(a, b, c)] = calc_avg_gs(potential_grid, a, b, c);
						delta += (potential_grid[GRID_INDEX(a, b, c)] - initial_value);
					}
				}
			}
		}
		printf("%i %f\n", i, delta);
		if (delta <= 0.001) break;
	}

	return 0;
}

int calc_potential_j(potential_grid_t potential_grid1, potential_grid_t potential_grid2, obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations)
{
	if (iterations & 1) {
		return -1;
	}

	potential_grid_t rf /* read from */ = potential_grid1;
	potential_grid_t wt /* write to */ = potential_grid2;

	// Jacobi
	for (uint32_t i = 0; i < iterations; i++) {
		for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
			for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
				for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
					if (position_is_goal(goal_position, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(obstacles_grid, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						wt[GRID_INDEX(a, b, c)] = calc_avg_gs(rf, a, b, c);
					}
				}
			}
		}

		potential_grid_t const tmp = rf; rf = wt; wt = tmp;
	}

	return 0;
}

static pthread_barrier_t barrier;
static uint32_t it, its;
static obstacles_grid_t og;
static position_t gp;
static potential_grid_t rf, wt;

static void* pj_work(void *arg)
{
	uint32_t const id = (uint32_t) arg;

	// Parallel Jacobi
	while (it < its) {
		uint64_t a, b, c;

		for (a = WORLD_SIZE_X / THREADS * id; a < WORLD_SIZE_X / THREADS * (id + 1); a++) {
			for (b = 0; b < WORLD_SIZE_Y; b++) {
				for (c = 0; c < WORLD_SIZE_Z; c++) {
					if (position_is_goal(gp, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(og, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						wt[GRID_INDEX(a, b, c)] = calc_avg_gs(rf, a, b, c);
					}
				}
			}
		}

		pthread_barrier_wait(&barrier);
		if (id == 0) {
			potential_grid_t tmp = rf; rf = wt; wt = tmp;
			it++;
			/*
			 * Note: I'm pretty sure this in not 100% portable; in some
			 * archs (e.g., alpha) there will be no visibility guarantee
			 * over i. It works well on IA32 and PowerPC, so...
			 */

		}
		pthread_barrier_wait(&barrier);
	}

	return 0;
}

int calc_potential_pj(potential_grid_t potential_grid1, potential_grid_t potential_grid2,
			obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations)
{
	if (iterations & 1) {
		return -1;
	}

	its = iterations;
	gp[0] = goal_position[0];
	gp[1] = goal_position[1];
	gp[2] = goal_position[2];
	og = obstacles_grid;

	rf /* read from */ = potential_grid1;
	wt /* write to */ = potential_grid2;

	pthread_barrierattr_t barrier_attr;
	pthread_barrierattr_init(&barrier_attr);
	pthread_barrier_init(&barrier, &barrier_attr, THREADS);

	pthread_t threads[THREADS];
	pthread_attr_t threads_attr[THREADS];

	for (uint32_t xpto = 1; xpto < THREADS; xpto++) {
		pthread_attr_init(&threads_attr[xpto]);
		pthread_create(&threads[xpto], &threads_attr[xpto], &pj_work, (void*) xpto);
	}
	pj_work((void*) 0);
	for (uint32_t xpto = 1; xpto < THREADS; xpto++) {
		void **trash = NULL;
		pthread_join(threads[xpto], trash);
	}

	return 0;
}


int find_waypoints(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid, position_t starting_position, position_t goal_position)
{
	position_t current_position;
	memcpy(current_position, starting_position, sizeof(position_t));

	for (int i = 0; i < 1024; i++) {
		potential_grid_cell_t best_value = potential_grid[GRID_INDEX(current_position[0], current_position[1], current_position[2])]; // So far best weigt == current weight
		position_t next; // Next position

		for (int8_t a = -1; a <= 1; a++) {
			if (current_position[0] + a >= 0 && current_position[0] + a < WORLD_SIZE_X) {
				for (int8_t b = -1; b <= 1; b++) {
					if (current_position[1] + b >= 0 && current_position[1] + b < WORLD_SIZE_Y) {
						for (int8_t c = -1; c <= 1; c++) {
							if (current_position[2] + c >= 0 && current_position[2] + c < WORLD_SIZE_Z) {
								if (!position_is_obstacle(obstacles_grid, current_position[0] + a, current_position[1] + b, current_position[2] + c)) {
									potential_grid_cell_t const new_value = potential_grid[GRID_INDEX(current_position[0] + a, current_position[1] + b, current_position[2] + c)];

									if (new_value > best_value) {
										best_value = new_value;
										next[0] = current_position[0] + a;
										next[1] = current_position[1] + b;
										next[2] = current_position[2] + c;
									}
								}
							}
						}
					}
				}
			}
		}

		printf("Next waypoint: [%lu, %lu, %lu] with value %2.12f (%e)\n", next[0],
				next[1], next[2], best_value, best_value);

		if (position_is_goal(goal_position, next[0], next[1], next[2])) {
			break;
		} else {
			// Set found waypoint as next position
			current_position[0] = next[0];
			current_position[1] = next[1];
			current_position[2] = next[2];
		}

	}

	return 0;
}
