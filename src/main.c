#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "algo.h"
#include "bitmap.h"
#include "config.h"
#include "heightmap.h"
#include "log.h"

ALGO ALGORITHM;
uint32_t ITERATIONS, THREADS;

map_t MAP;
bitmap_t OBSTACLES;
uint8_t HEIGHT_LIMIT;
uint64_t WORLD_SIZE_X, WORLD_SIZE_Y, WORLD_SIZE_Z;
position_t STARTING_POINT, GOAL;

int main(int argc, char * argv[])
{
	// TODO: parse arguments

	ALGORITHM = GS;
	ITERATIONS = 500;
	THREADS = 4;

	WORLD_SIZE_X = 64;
	WORLD_SIZE_Y = 64;
	WORLD_SIZE_Z = 256;
	HEIGHT_LIMIT = 174;

	STARTING_POINT[0] = 3; STARTING_POINT[1] = 3; STARTING_POINT[2] = heightmap[3][3] + 2;
	GOAL[0] = WORLD_SIZE_X - 3; GOAL[1] = WORLD_SIZE_Y - 3; GOAL[2] = heightmap[WORLD_SIZE_X - 3][WORLD_SIZE_Y - 3] + 2;
	MAP = malloc(WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z * sizeof(map_cell_t));
	OBSTACLES = bitmap_create(WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z);

	logStart("initialize obstacle map");
	// Recode heightmap to global_obstacles
	for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
		for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
			uint8_t height = heightmap[a][b];

			// some regions cannot be flown over
			if (height > HEIGHT_LIMIT) height = UINT8_MAX;

			for (uint8_t c = 0; c < height; c++) {
				bitmap_set(OBSTACLES, a * WORLD_SIZE_X + b * WORLD_SIZE_Y + c);
			}
		}
	}
	logEnd("initialize obstacle map");

	logStart("initialize weightmap from obstacle map");
	for (uint64_t a = 0; a < WORLD_SIZE_X; a++) {
		for (uint64_t b = 0; b < WORLD_SIZE_Y; b++) {
			for (uint64_t c = 0; c < WORLD_SIZE_Z; c++) {
				if (is_goal(a, b, c)) {
					MAP[INDEX(a, b, c)] = WEIGHT_SINK;
				} else if (is_obstacle(a, b, c)) {
					MAP[INDEX(a, b, c)] = WEIGHT_OBSTACLE;
				} else {
					MAP[INDEX(a, b, c)] = WEIGHT_INIT;
				}
			}
		}
	}
	logEnd("initialize weightmap from obstacle map");

	calc_potential_j(MAP);
	find_waypoints(MAP);

	return 0;
}
