# 3D Path Planning

This is an implementation of a 3D Path Planner using the [Laplace operator](http://en.wikipedia.org/wiki/Laplacian).

There are two linear solvers under the hood: the Gauss-Seidel, and the Jacobi methods. Moreover, parallel implementations of both methods are featured. The Gauss-Seidel parallel implementation uses a pipeline scheme.

## Configuring

No command line arguments exist because this application is being tested in an embedded environment. Take a look at the `src/config.h` file. I believe it is more or less self explainatory :-)
