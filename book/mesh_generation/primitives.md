# Primitives

Besides the `ToTriMesh` and `ToLinestrip` traits, the `procedural` module
exports free functions that generates various meshes, including those
accessible by the two former traits:

| Function         | Description |
|--                | --          |
| `circle(...)`    |             |
| `cone(...)`      |             |
| `cuboid(...)`    |             |
| `cylinder(...)`  |             |
| `quad(...)`      |             |
| `rectangle(...)` |             |
| `sphere(...)`    |             |


Those shapes are usually created by moving and stretching their unit version. A
_unit_ geometry is a geometry that has its principal attributes set to one:

| Function              | Description |
|--                     | --          |
| `unit_circle(...)`    | Creates a planar circle with a diameter of 1.0.   |
| `unit_cone(...)`      | Creates a cone with a diameter and height of 1.0. |
| `unit_cuboid(...)`    | Creates a cuboid (rectangle or box) with extents along each axis equal to 1.0. |
| `unit_cylinder(...)`  | Creates a cylinder with a diameter and height of 1.0. |
| `unit_quad(...)`      |             |
| `unit_rectangle(...)` |             |
| `unit_sphere(...)`    |             |

## Parametric surfaces

| Function                                    | Description |
| --                                          | --          |
| bezier_curve(...)                           |             |
| bezier_surface(...)                         |             |
| rational_bezier_curve(...)                  |             |
| rational_bezier_surface(...)                |             |

## Convex hull

Finally, given a set of points, **ncollide** is able to compute their convex
hulls using QuickHull algorithm which has an average $$O(n \log{n})$$
complexity. Because computing the convex hull of 2d and 3d geometries is so
common no matter the number of dimensions you chose to compile **ncollide**
for, two function are always available:

| Function             | Description                                     |
| --                   | --                                              |
| `convex_hull2d(...)` | Computes the convex hull of a set of 2D points. |
| `convex_hull3d(...)` | Computes the convex hull of a set of 3D points. |

If you are not interested on the `TriMesh` representation of the convex hull,
but only on the indices of the vertices contained by the convex hull, use the
`_idx` variant of those functions, namely `convex_hull2d_idx(...)` and
`convex_hull3d_idx(...)`.

## Example
Le following example creates a Bézier surface (curve in 2D) and computes its
convex hull.
