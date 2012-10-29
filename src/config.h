#ifndef CONFIG_H_
#define CONFIG_H_

#define WEIGHT_OBSTACLE 0.0
#define WEIGHT_INIT 0.0
#define WEIGHT_SINK 1.0

#include <stdint.h>
#include "bitmap.h"

typedef enum { GS, PGS, J, PJ } ALGO;
typedef uint64_t position_t[3];
typedef double map_cell_t;
typedef double * map_t;

extern ALGO ALGORITHM;
extern uint32_t ITERATIONS, THREADS;

extern map_t MAP;
#define INDEX(x, y, z) ((z) * WORLD_SIZE_X * WORLD_SIZE_Y + (y) * WORLD_SIZE_Y + (x))
extern bitmap_t OBSTACLES;
extern uint8_t HEIGHT_LIMIT;
extern uint64_t WORLD_SIZE_X, WORLD_SIZE_Y, WORLD_SIZE_Z;
extern position_t STARTING_POINT, GOAL;

#endif /* CONFIG_H_ */
