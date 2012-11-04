#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "bitmap.h"
#include "config.h"
#include "maps.h"

int obstacles_grid_create(obstacles_grid_t *obstacles_grid, obstacles_grid_cell_t *cells, size_t n)
{
	bitmap_create_static(obstacles_grid, cells, n);

	return 0;
}

bool position_is_goal(position_t goal_position, uint64_t x, uint64_t y, uint64_t z)
{
	return (goal_position[0] == x) && (goal_position[1] == y) && (goal_position[2] == z);
}

bool position_is_obstacle(obstacles_grid_t obstacles_grid, uint64_t x, uint64_t y, uint64_t z)
{
	return bitmap_is_set(&obstacles_grid, GRID_INDEX(x, y, z));
}
