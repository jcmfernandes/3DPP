#ifndef CONFIG_H_
#define CONFIG_H_

#define HEIGHT_LIMIT 174
#define WORLD_SIZE_X 64
#define WORLD_SIZE_Y 64
#define WORLD_SIZE_Z 256
#define STARTING_POINT_X 1
#define STARTING_POINT_Y 1
#define GOAL_X 63
#define GOAL_Y 63

/* Algorithms:
 * GS - Gauss-Seidel
 * J - Jacobi
 * CGS - Convergence delta Gauss-Seidel
 * PGS - Parallel Gauss-Seidel
 * PJ - Parallel Jacobi
 */
#define ALGORITHM PGS

/*
 * GS settings
 */
#define GS_ITERATIONS 256

/*
 * CGS settings
 */
#define CGS_CONVERGENCE_DELTA 0.001

/*
 * PGS settings
 */
#define PGS_ITERATIONS 256
#define PGS_X_COMPARTMENTS 2
#define PGS_Y_COMPARTMENTS 2
#define PGS_Z_COMPARTMENTS 2

/*
 * J setttings
 */
#define J_ITERATIONS 256

/*
 * PJ settings
 */
#define PJ_ITERATIONS 256
#define PJ_THREADS 4

#endif /* CONFIG_H_ */
