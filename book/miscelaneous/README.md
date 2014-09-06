# Miscelanous

Besides the main collision detection and mesh generation related features,
**ncollide** exposes a number of unclassifiable geometric operations that are
used internally by the library. Those operations are exported by the `utils`
module. Here are listed the most useful operations, most of them being
dimension-generic (no matter which version of **ncollide** you use). See [the
documentation](http://ncollide.org/doc/ncollide3df32/utils/index.html) for
more.


| Function                    | Description                                          |
|--                           | --                                                   |
| `center(pts)`               | Computes the center of the points `pts`.             |
| `circumcircle(a, b, c)`     | Computes the circumcircle of the triangle `a, b, c`. |
| `cov(pts)`                  | Computes the covariance matrix of the points `pts`.  |
| `is_point_in_triangle(...)` | Tests that a point is inside of a triangle.          |
| `sort3(a, b, c)`            | Sorts in increasing order a set of three values.     |
| `triangulate(pts)`          | Triangulates the points `pts`.                       |


Some optimization-related functions are also implemented:

| Function      | Description                                          |
|--             | --                                                   |
| `newton(...)` | Finds a root of a function using the Newton method.  |
| `bfgs(...)`   | Minimizes a function using the BFGS method.          |
