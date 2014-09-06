## Ray Casting traits

The `ray::RayCast` trait are implemented by geometric primitives that can be
intersected by a ray:

| Field | Description                               |
|--     | --                                        |
| `.toi_and_normal_with_ray(ray, is_solid)`                         |    |
| `.toi_with_ray(ray, is_solid)`                                    |    |
| `.toi_and_normal_and_uv_with_ray(ray, is_solid)`                  |    |
| `.intersects_ray(ray)`                                            |    |
| `.toi_with_transform_and_ray(m, ray, is_solid)`                   |    |
| `.toi_and_normal_with_transform_and_ray(m, ray, is_solid)`        |    |
| `.toi_and_normal_and_uv_with_transform_and_ray(m, ray, is_solid)` |    |
| `.intersects_with_transform_and_ray(m, ray)`                      |    |

Note that if you implement this trait for your own geometry, only the first
method is required, the other ones are automatically infered.
