#ifndef GRIDS_H_
#define GRIDS_H_

#include <stdbool.h>
#include <stdint.h>
#include "bitmap.h"

#define WEIGHT_OBSTACLE (0.0)
#define WEIGHT_INIT (0.0)
#define WEIGHT_SINK (1.0)

typedef uint64_t position_t[3];

typedef double potential_grid_cell_t;
typedef double * potential_grid_t;

typedef bitmap_cell_t obstacles_grid_cell_t;
typedef bitmap_t obstacles_grid_t;


#define GRID_INDEX(x, y, z) ((z) * WORLD_SIZE_X * WORLD_SIZE_Y + (y) * WORLD_SIZE_Y + (x))

#define obstacles_grid_required_cells(n) (bitmap_required_cells(n))
int obstacles_grid_create(obstacles_grid_t *obstacles_grid, obstacles_grid_cell_t *cells, size_t n);

bool position_is_goal(position_t goal_position, uint64_t x, uint64_t y, uint64_t z);
bool position_is_obstacle(obstacles_grid_t obstacles_grid, uint64_t x, uint64_t y, uint64_t z);
int position_copy(position_t dest, position_t src);

#endif /* GRIDS_H_ */
