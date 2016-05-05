# Miscellaneous

Besides the main collision detection and mesh generation related features,
**ncollide** exposes a number of unclassified operations that are used
internally by the library. Those operations are exported by the `utils` module.
Here are listed some of the most useful operations:

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
| `newton(...)` | Finds a root of a function using the [Newton](http://en.wikipedia.org/wiki/Newton's_method) method.  |
| `bfgs(...)`   | Minimizes a function using the [BFGS](http://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm) method. |

See the [API documentation](../rustdoc/ncollide/utils/index.html) for an
exhaustive list.
