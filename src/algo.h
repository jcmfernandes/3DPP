#ifndef ALGO_H_
#define ALGO_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

bool is_goal(uint64_t x, uint64_t y, uint64_t z);
bool is_obstacle(uint64_t x, uint64_t y, uint64_t z);

int calc_potential_gs(map_t map);
int calc_potential_j(map_t map);

int find_waypoints(map_t map);

#endif /* ALGO_H_ */
