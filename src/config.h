#ifndef CONFIG_H_
#define CONFIG_H_

/* Algorithms:
 * GS - Gauss-Seidel
 * J - Jacobi
 * PGS - Parallel Gauss-Seidel
 * PJ - Parallel Jacobi
 */
#define ALGORITHM PJ
#define THREADS 4
#define ITERATIONS 1024

#define HEIGHT_LIMIT 174
#define WORLD_SIZE_X 64
#define WORLD_SIZE_Y 64
#define WORLD_SIZE_Z 256
#define STARTING_POINT_X 1
#define STARTING_POINT_Y 1
#define GOAL_X 63
#define GOAL_Y 63

#endif /* CONFIG_H_ */
