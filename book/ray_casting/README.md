# Ray casting

Ray casting is also one of the core geometric queries in the field of collision
detection. Besides the fact it can be used for rendering (using ray-tracing
like methods), it is useful for e.g. [continuous collision
detection](../collision_detection/time_of_impact.html) and navigation on a
virtual environment. Therefore **ncollide** has efficient ray casting
algorithms for arbitrary
[support-mapped](../geometric_representations/README.html) convex geometries,
triangle meshes (in 3D), and line strips (in 2D).


The main ray-casting related data structure is obviously `ray::Ray` itself:

| Field  | Description                    |
|--      | --                             |
| `orig` | The ray starting point.        |
| `dir`  | The ray propagation direction. |


The result of a successful ray-cast is given by the `ray::RayIntersection`
structure:

| Field         | Description                               |
|--             | --                                        |
| `toi`         | The _time of impact_ of the ray on the object. Using this the exact intersection point can be computed using: `ray.orig + ray.dir * result.toi` |
| `normal` | The normal of the hit shape at the intersection point.  |
| `uvs`    | If available, the texture coordinates of the hit shape at the intersection point. If the texture coordinates information is not computable, this is set to `None`. |

##### Beware the margins!

Do not forget that **ncollide** geometries (excluding bounding volumes) have a
[margin](../geometric_representations/README.html#margins). It is taken in
account when casting rays! This can significantly slow down the ray-casting
operation: for example, casting a ray on a triangle mesh with a non-zero margin
is much slower than on a mesh without margin. Remember that, most of the time,
creating a geometry without the default margin is done using the corresponding
static method `::new_with_margin(..., 0.0)`, the zero margin being the last
argument. Refer to the [geometry
definitions](../geometric_representations/simple_geometries.html) for more
details.
