# Ray casting

Ray casting is also one of the core geometric queries in the field of collision
detection. Besides the fact it can be used for rendering (using ray-tracing
like methods), it is useful for e.g. continuous collision detection and
navigation on a virtual environment. Therefore **ncollide** has efficient ray
casting algorithms for arbitrary
[support-mapped](../geometric_representations/README.html#support-map) convex
shapes, triangle meshes (in 3D), and line strips (in 2D).


The main ray-casting related data structure is obviously `ray::Ray` itself:

| Field  | Description                    |
|--      | --                             |
| `orig` | The ray starting point.        |
| `dir`  | The ray propagation direction. |


The result of a successful ray-cast is given by the `ray::RayIntersection`
structure:

| Field         | Description                               |
|--             | --                                        |
| `toi`         | The _time of impact_ of the ray on the object. Using this, the exact intersection point can be computed using: `ray.orig + ray.dir * result.toi`. |
| `normal` | The normal at the intersection point of the shape hit by the ray.  |
| `uvs`    | If available, the texture coordinates at the intersection point of the shape hit by the ray. If the texture coordinates information is not computable, this is set to `None`. |

## Traits

The `ray::RayCast` trait is implemented by shapes that can be intersected by a
ray:

| Method | Description |
|--      | --          |
| `.toi_and_normal_with_ray(ray, solid)`                 | Computes the point and normal of the intersection between `ray` and `self`. |
| `.toi_with_ray(ray, solid)`                            | Computes the point of the intersection between `ray` and `self`. |
| `.toi_and_normal_and_uv_with_ray(ray, solid)`          | Computes the point, normal, and texture coordinates of the intersection between `ray` and `self`. |
| `.intersects_ray(ray)`                                 | Tests whether `ray` intersects `self`. |
| `.toi_with_transform_and_ray(m, ray, solid)`           | Computes the point and normal of the intersection between `ray` and `self` transformed by `m`. |
| `.toi_and_normal_with_transform_and_ray(m, ray, solid)`| Computes the point of the intersection between `ray` and `self` transformed by `m`. |
| `.toi_and_normal_and_uv_with_transform_and_ray(...)`   | Computes the point, normal, and texture coordinates of the intersection between `ray` and `self` transformed by `m`. |
| `.intersects_with_transform_and_ray(m, ray)`           | Tests whether `ray` intersects `self` transformed by `m`. |

Note that if you implement this trait for your own shape, only the first
method is required. The other ones are automatically inferred (but for
optimization purpose you might want to specialize the other methods too).

More details about the `solid` flag can be found in
[Solid ray cast](../ray_casting/solid_ray_cast.html).
