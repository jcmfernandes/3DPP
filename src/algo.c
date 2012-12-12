#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
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

int calc_potential_j(potential_grid_t potential_grid1, potential_grid_t potential_grid2,
		obstacles_grid_t obstacles_grid, position_t goal_position, uint32_t iterations)
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

typedef struct {
	pthread_barrier_t barrier;
	pthread_mutex_t lock;
	uint32_t n_threads, tid_counter, iteration, total_number_iterations;
	obstacles_grid_t obstacles_grid;
	position_t goal_position;
	potential_grid_t potential_grid1, potential_grid2;
} pj_context_t;

static void* pj_work(void *arg)
{
	pj_context_t *const context = (pj_context_t*) arg;

	pthread_mutex_lock(&context->lock);
	uint32_t const tid = context->tid_counter++;
	pthread_mutex_unlock(&context->lock);
	potential_grid_t rf /* read from */ = context->potential_grid1;
	potential_grid_t wt /* write to */ = context->potential_grid2;

	pthread_barrier_wait(&context->barrier);

	uint64_t const x_left_limit = WORLD_SIZE_X / context->n_threads * tid;
	uint64_t const x_right_limit = WORLD_SIZE_X / context->n_threads * (tid + 1);
	// [x_left_limit, x_right_limit[

	// Parallel Jacobi
	while (context->iteration < context->total_number_iterations) {
		uint64_t a, b, c;

		for (a = x_left_limit; a < x_right_limit; a++) {
			for (b = 0; b < WORLD_SIZE_Y; b++) {
				for (c = 0; c < WORLD_SIZE_Z; c++) {
					if (position_is_goal(context->goal_position, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(context->obstacles_grid, a, b, c)) {
						wt[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						wt[GRID_INDEX(a, b, c)] = calc_avg_gs(rf, a, b, c);
					}
				}
			}
		}

		potential_grid_t const tmp = rf;
		rf = wt;
		wt = tmp;
		if (tid == 0) {
			context->iteration++;
			/*
			 * Note: I'm pretty sure this in not 100% portable; in some
			 * archs (e.g., alpha) there will be no visibility guarantee
			 * over "context->iteration". It works well on IA32 and PowerPC, so...
			 */

		}
		pthread_barrier_wait(&context->barrier);
	}

	return 0;
}

int calc_potential_pj(potential_grid_t potential_grid1, potential_grid_t potential_grid2,
			obstacles_grid_t obstacles_grid, position_t goal_position,
			uint32_t iterations, uint32_t n_threads)
{
	if (iterations & 1) {
		return -1;
	}

	pj_context_t context;
	context.potential_grid1 = potential_grid1;
	context.potential_grid2 = potential_grid2;
	context.obstacles_grid = obstacles_grid;
	position_copy(context.goal_position, goal_position);
	context.total_number_iterations = iterations;
	context.iteration = 0;
	context.n_threads = n_threads;
	context.tid_counter = 0;

	pthread_mutexattr_t lock_attr;
	pthread_mutexattr_init(&lock_attr);
	pthread_mutex_init(&context.lock, &lock_attr);

	pthread_barrierattr_t barrier_attr;
	pthread_barrierattr_init(&barrier_attr);
	pthread_barrier_init(&context.barrier, &barrier_attr, n_threads);

	pthread_t threads[n_threads];
	pthread_attr_t threads_attr[n_threads];
	for (uint32_t i = 1; i < n_threads; i++) {
		pthread_attr_init(&threads_attr[i]);
		pthread_create(&threads[i], &threads_attr[i], &pj_work, (void*) &context);
	}
	pj_work((void*) &context);
	for (uint32_t i = 1; i < n_threads; i++) {
		void **trash = NULL;
		pthread_join(threads[i], trash);
	}

	return 0;
}


int find_waypoints(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid, position_t starting_position, position_t goal_position)
{
	position_t current_position;
	position_copy(current_position, starting_position);

	for (int i = 0; i < 1024; i++) {
		potential_grid_cell_t best_value = potential_grid[GRID_INDEX(current_position[0], current_position[1], current_position[2])]; // So far best weigt == current weight
		position_t next; // Next position

		for (int8_t a = -1; a <= 1; a++) {
			if (current_position[0] + a >= 0 && current_position[0] + a < WORLD_SIZE_X) {
				for (int8_t b = -1; b <= 1; b++) {
					if (current_position[1] + b >= 0 && current_position[1] + b < WORLD_SIZE_Y) {
						for (int8_t c = -1; c <= 1; c++) {
							if (current_position[2] + c >= 0 && current_position[2] + c < WORLD_SIZE_Z) {
								if (! position_is_obstacle(obstacles_grid, current_position[0] + a, current_position[1] + b, current_position[2] + c)) {
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
			position_copy(current_position, next);
		}

	}

	return 0;
}
