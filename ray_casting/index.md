# Ray casting

Ray casting is also one of the core geometric queries in the field of collision
detection. Besides the fact it can be used for rendering (using ray-tracing
like methods), it is useful for e.g. continuous collision detection and
navigation on a virtual environment. Therefore **ncollide** has efficient ray
casting algorithms for all the shapes it implements (including functions that
are able to cast rays on arbitrary
[support-mapped](../geometric_representations/index.html#support-map) convex
shapes).


The main ray-casting related data structure is obviously `ray::Ray` itself:

| Field  | Description                    |
|--      | --                             |
| `orig` | The ray starting point.        |
| `dir`  | The ray propagation direction. |


The result of a successful ray-cast is given by the `ray::RayIntersection`
structure:

| Field    | Description                               |
|--        | --                                        |
| `toi`    | The _time of impact_ of the ray on the object. |
| `normal` | The normal (in absolute coordinates) at the intersection point of the shape hit by the ray.  |
| `uvs`    | If available, the texture coordinates at the intersection point of the shape hit by the ray. If the texture coordinates information is not computable, this is set to `None`. |

Recall that the exact point of intersection may be computed from the
 _time of impact_ with the following formula: `ray.orig + ray.dir * result.toi`.
A physical intepretation of the time of impact is the time needed for a point
with velocity `ray.dir` to travel from the position `ray.orig` to the object
hit.

## Traits

The `ray::RayCast` trait is implemented by shapes that can be intersected by a
ray:

| Method | Description |
|--      | --          |
| `.toi_with_ray(m, ray, solid)`                   | Computes the time of impact of the intersection between `ray` and `self` transformed by `m`. |
| `.toi_and_normal_with_ray(m, ray, solid)`        | Computes the time of impact and normal of the intersection between `ray` and `self` transformed by `m`. |
| `.toi_and_normal_and_uv_with_ray(m, ray, solid)` | Computes the time of impact , normal, and texture coordinates of the intersection between `ray` and `self` transformed by `m`. |
| `.intersects_ray(m, ray)`                        | Tests whether `ray` intersects `self` transformed by `m`. |

Note that if you implement this trait for your own shape, only the second method
of this list (namely `.toi_and_normal_with_ray(...)` is required. The other
ones are automatically inferred (but for optimization purpose you might want to
specialize the other methods as well).

More details about the `solid` flag can be found in
[Solid ray cast](../ray_casting/solid_ray_cast.html).
