#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "bitmap.h"
#include "config.h"
#include "log.h"

bool is_goal(uint64_t x, uint64_t y, uint64_t z)
{
	return (GOAL[0] == x) && (GOAL[1] == y) && (GOAL[2] == z);
}

bool is_obstacle(uint64_t x, uint64_t y, uint64_t z)
{
	return bitmap_is_set(OBSTACLES, INDEX(x, y, z));
}

static map_cell_t calc_avg_gs(map_t map, uint64_t x, uint64_t y, uint64_t z)
{
	uint8_t counter = 6;
	map_cell_t value = 0.0;

	if (x > 0 && x < WORLD_SIZE_X - 1) {
		value += map[INDEX(x - 1, y, z)];
		value += map[INDEX(x + 1, y, z)];
	} else {
		if (x > 0) {
			value += map[INDEX(x - 1, y, z)];
			counter--;
		}
		if (x < WORLD_SIZE_X - 1) {
			value += map[INDEX(x + 1, y, z)];
			counter--;
		}
	}

	if (y > 0 && y < WORLD_SIZE_Y - 1) {
		value += map[INDEX(x, y - 1, z)];
		value += map[INDEX(x, y + 1, z)];
	} else {
		if (y > 0) {
			value += map[INDEX(x, y - 1, z)];
			counter--;
		}
		if (y < WORLD_SIZE_Y - 1) {
			value += map[INDEX(x, y + 1, z)];
			counter--;
		}
	}

	if (z > 0 && z < WORLD_SIZE_Z - 1) {
		value += map[INDEX(x, y, z - 1)];
		value += map[INDEX(x, y, z + 1)];
	} else {
		if (z > 0) {
			value += map[INDEX(x, y, z - 1)];
			counter--;
		}
		if (z < WORLD_SIZE_Z - 1) {
			value += map[INDEX(x, y, z + 1)];
			counter--;
		}
	}

	value /= counter;

	return value;
}

int calc_potential_gs(map_t map)
{
	// Gauss-Seidel
	for (uint32_t i = 0; i < ITERATIONS; i++) {
		for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
			for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
				for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
					if (is_goal(a, b, c)) {
						map[INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (is_obstacle(a, b, c)) {
						map[INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						map[INDEX(a, b, c)] = calc_avg_gs(map, a, b, c);
					}
				}
			}
		}
	}

	return 0;
}

int calc_potential_j(map_t map)
{
	map_t rf /* read from */ = map;
	map_t wt /* write to */ = malloc(WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z * sizeof(map_cell_t));

	// Jacobi
	for (uint32_t i = 0; i < ITERATIONS; i++) {
		for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
			for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
				for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
					if (is_goal(a, b, c)) {
						wt[INDEX(a, b, c)] = WEIGHT_SINK;
					} else if (is_obstacle(a, b, c)) {
						wt[INDEX(a, b, c)] = WEIGHT_OBSTACLE;
					} else {
						wt[INDEX(a, b, c)] = calc_avg_gs(rf, a, b, c);
					}
				}
			}
		}

		map_t tmp = rf; rf = wt; wt = tmp;
	}

	free(wt);

	return 0;
}

int find_waypoints(map_t map)
{
	position_t current_position = { STARTING_POINT[0], STARTING_POINT[1], STARTING_POINT[2] };

	logStart("findWaypoints");

	for (int i = 0; i < 1024; i++) {
		map_cell_t best_value = map[INDEX(current_position[0], current_position[1], current_position[2])]; // So far best weigt == current weight
		position_t next; // Next position

		for (int8_t a = -1; a <= 1; a++) {
			if (current_position[0] + a >= 0 && current_position[0] + a < WORLD_SIZE_X) {
				for (int8_t b = -1; b <= 1; b++) {
					if (current_position[1] + b >= 0 && current_position[1] + b < WORLD_SIZE_Y) {
						for (int8_t c = -1; c <= 1; c++) {
							if (current_position[2] + c >= 0 && current_position[2] + c < WORLD_SIZE_Z) {
								if (!is_obstacle(current_position[0] + a, current_position[1] + b, current_position[2] + c)) {
									map_cell_t new_value = map[INDEX(current_position[0] + a, current_position[1] + b, current_position[2] + c)];

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

		if (is_goal(next[0], next[1], next[2]))
			break;
		else {
			// Set found waypoint as next position
			current_position[0] = next[0];
			current_position[1] = next[1];
			current_position[2] = next[2];
		}

	}

	logEnd("findWaypoints");

	return 0;
}
