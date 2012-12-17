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

int calc_potential_gs_conv(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t goal_position, double convergence)
{
	// Gauss-Seidel
	for (;;) {
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

		if (delta <= convergence) break;
	}

	return 0;
}

typedef struct {
	uint32_t stage;
	pthread_cond_t signal;
	pthread_mutex_t lock;
} pgs_pipeline_context_t;

typedef struct {
	pgs_pipeline_context_t *pipeline_contexts;
	pthread_mutex_t lock;
	uint32_t tid_counter;
	uint32_t total_number_iterations;
	uint32_t x_compartments, y_compartments, z_compartments;
	obstacles_grid_t obstacles_grid;
	position_t goal_position;
	potential_grid_t potential_grid;
} pgs_context_t;

static void* pgs_work(void *arg)
{
	pgs_context_t *const context = (pgs_context_t*) arg;

	pthread_mutex_lock(&context->lock);
	uint32_t const tid = context->tid_counter++;
	pthread_mutex_unlock(&context->lock);

	uint32_t const x = tid % context->x_compartments;
	uint32_t const y = (tid / context->x_compartments) % context->y_compartments;
	uint32_t const z = (tid / (context->y_compartments * context->x_compartments)) % context->z_compartments;

	uint64_t const x_left_limit = WORLD_SIZE_X / context->x_compartments * x;
	uint64_t const x_right_limit = x_left_limit + WORLD_SIZE_X / context->x_compartments;
	// [x_left_limit, x_right_limit[
	uint64_t const y_left_limit = WORLD_SIZE_Y / context->y_compartments * y;
	uint64_t const y_right_limit = y_left_limit + WORLD_SIZE_Y / context->y_compartments;
	// [y_left_limit, y_right_limit[
	uint64_t const z_left_limit = WORLD_SIZE_Z / context->z_compartments * z;
	uint64_t const z_right_limit = z_left_limit + WORLD_SIZE_Z / context->z_compartments;
	// [z_left_limit, z_right_limit[

	// Parallel Gauss-Seidel
	for (uint32_t iteration = 0; iteration < context->total_number_iterations;) {

		if (x > 0) { // we need to wait for the previous pipeline stage to finish
			uint32_t const i = z * context->x_compartments * context->y_compartments + y * context->y_compartments + (x - 1);
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage <= iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}
		if (y > 0) { // we need to wait for the previous pipeline stage to finish
			uint32_t const i = z * context->x_compartments * context->y_compartments + (y - 1) * context->y_compartments + x;
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage <= iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}
		if (z > 0) { // we need to wait for the previous pipeline stage to finish
			uint32_t const i = (z - 1) * context->x_compartments * context->y_compartments + y * context->y_compartments + x;
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage <= iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}

		for (uint64_t a = x_left_limit; a < x_right_limit; a++) {
			for (uint64_t b = y_left_limit; b < y_right_limit; b++) {
				for (uint64_t c = z_left_limit; c < z_right_limit; c++) {
					if (position_is_goal(context->goal_position, a, b, c)) {
						context->potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (position_is_obstacle(context->obstacles_grid, a, b, c)) {
						context->potential_grid[GRID_INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						context->potential_grid[GRID_INDEX(a, b, c)] = calc_avg_gs(context->potential_grid, a, b, c);
					}
				}
			}
		}

		iteration++;
		{
			uint32_t const i = z * context->x_compartments * context->y_compartments + y * context->y_compartments + x;
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			pipeline_context->stage = iteration;
			pthread_cond_broadcast(&pipeline_context->signal);
			pthread_mutex_unlock(&pipeline_context->lock);
		}

		if (x + 1 < context->x_compartments) { // we need to wait for the next pipeline stage to finish
			uint32_t const i = z * context->x_compartments * context->y_compartments + y * context->y_compartments + (x + 1);
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage < iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}
		if (y + 1 < context->y_compartments) { // we need to wait for the next pipeline stage to finish
			uint32_t const i = z * context->x_compartments * context->y_compartments + (y + 1) * context->y_compartments + x;
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage < iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}
		if (z + 1 < context->z_compartments) { // we need to wait for the next pipeline stage to finish
			uint32_t const i = (z + 1) * context->x_compartments * context->y_compartments + y * context->y_compartments + x;
			pgs_pipeline_context_t *pipeline_context = &context->pipeline_contexts[i];
			pthread_mutex_lock(&pipeline_context->lock);
			while (pipeline_context->stage < iteration) pthread_cond_wait(&pipeline_context->signal, &pipeline_context->lock);
			pthread_mutex_unlock(&pipeline_context->lock);
		}
	}

	return 0;
}

int calc_potential_pgs(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t goal_position,uint32_t iterations, uint32_t x_compartments,
		uint32_t y_compartments, uint32_t z_compartments)
{
	if (iterations & 1) {
		return -1;
	}

	int return_code = 0;

	pgs_context_t context;
	pgs_pipeline_context_t pipeline_contexts[x_compartments * y_compartments * z_compartments];
	context.pipeline_contexts = pipeline_contexts;
	context.potential_grid = potential_grid;
	context.obstacles_grid = obstacles_grid;
	position_copy(context.goal_position, goal_position);
	context.total_number_iterations = iterations;
	context.x_compartments = x_compartments;
	context.y_compartments = y_compartments;
	context.z_compartments = z_compartments;
	context.tid_counter = 0;

	uint32_t const n_threads = x_compartments * y_compartments * z_compartments;

	pthread_mutexattr_t lock_attr;
	pthread_mutexattr_init(&lock_attr);
	pthread_condattr_t cond_attr;
	pthread_condattr_init(&cond_attr);
	pthread_mutex_init(&context.lock, &lock_attr);
	for (uint32_t i = 0; i < n_threads; i++) {
		pipeline_contexts[i].stage = 0;
		pthread_cond_init(&pipeline_contexts[i].signal, &cond_attr);
		pthread_mutex_init(&pipeline_contexts[i].lock, &lock_attr);
	}

	pthread_t threads[n_threads];
	pthread_attr_t threads_attr[n_threads];
	for (uint32_t i = 1; i < n_threads; i++) {
		pthread_attr_init(&threads_attr[i]);
		pthread_create(&threads[i], &threads_attr[i], &pgs_work, (void*) &context);
	}
	pgs_work((void*) &context);
	for (uint32_t i = 1; i < n_threads; i++) {
		void *thread_return_code;
		pthread_join(threads[i], &thread_return_code);
		if (thread_return_code != 0) return_code = -1;
	}

	return return_code;
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
	uint32_t n_threads, tid_counter, total_number_iterations;
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

	uint64_t const x_left_limit = WORLD_SIZE_X / context->n_threads * tid;
	uint64_t const x_right_limit = WORLD_SIZE_X / context->n_threads * (tid + 1);
	// [x_left_limit, x_right_limit[

	// Parallel Jacobi
	for (uint32_t iteration = 0; iteration < context->total_number_iterations; iteration++) {
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

		potential_grid_t const tmp = rf; rf = wt; wt = tmp;
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

	int return_code = 0;

	pj_context_t context;
	context.potential_grid1 = potential_grid1;
	context.potential_grid2 = potential_grid2;
	context.obstacles_grid = obstacles_grid;
	position_copy(context.goal_position, goal_position);
	context.total_number_iterations = iterations;
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
		void *thread_return_code;
		pthread_join(threads[i], &thread_return_code);
		if (thread_return_code != 0) return_code = -1;
	}

	return return_code;
}


int find_waypoints(potential_grid_t potential_grid, obstacles_grid_t obstacles_grid,
		position_t starting_position, position_t goal_position, FILE *stream)
{
	position_t current_position;
	position_copy(current_position, starting_position);

	for (int i = 0; i < 1024; i++) {
		potential_grid_cell_t best_value = potential_grid[GRID_INDEX(current_position[0], current_position[1], current_position[2])]; // So far best weight == current weight
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

		fprintf(stream, "[%lu, %lu, %lu]\n", next[0], next[1], next[2]);

		if (position_is_goal(goal_position, next[0], next[1], next[2])) {
			break;
		} else {
			// Set found waypoint as next position
			position_copy(current_position, next);
		}

	}

	return 0;
}
