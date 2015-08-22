# Bounding Sphere

The `bounding_volume::BoundingSphere` is a sphere that contains completely the
bounded shape.

<center>
![bounding sphere](../img/bounding_volume_bounding_sphere.svg)
</center>

Being an unorientable shape, it is fully defined by its center and its
radius:

| Method      | Description                                                    |
|--           | --                                                             |
| `.center()` | The bounding sphere center. |
| `.radius()` | The bounding sphere radius. |


Of course, the bounding sphere implements the `BoundingVolume` trait. The
following shows the effect of the `.loosen(m)` method on it:

<center>
![loose bounding sphere](../img/bounding_volume_bounding_sphere_loose.svg)
</center>

Finally, note that a bounding sphere supports ray casting and point queries as described
by the [RayCast](../ray_casting/index.html) and
[PointQuery](../point_query/index.html) traits.

## Creating a Bounding Sphere

There are three ways to create a bounding sphere. The main one is to use the usual
static method `BoundingSphere::new(center, radius)`.

The second is to use the `bounding_volume.bounding_sphere(g, m)` function,
where `g` and `m` are the shape and its position (e.g. a transformation
matrix).

While this is not recommended except for generic programming, you may as well
directly call the method from the `bounding_volume::HasBoundingVolume` trait
implemented by any shape of `ncollide`:

| Method                | Description                                                |
|--                     | --                                                         |
| `.bounding_volume(m)` | Computes the bounding sphere of `self` transformed by `m`. |

While using the trait method directly works (this is actually what
`bounding_volume.bounding_sphere(...)` does under the hood), the compiler might
sometimes fail to infer correctly the types involved in the trait
implementation and output a cryptic error message. Also note that the
`HasBoundingVolume` trait actually takes the bounding volume type as a type
parameter. Therefore, you may have to specify explicitly the return type of
the method in order to use it, e.g. `let bs: BoundingSphere<Pnt3<f32>> =
g.bounding_volume(m);`.

## Example

The following example computes the bounding spheres of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder bounding
sphere, and performs some tests.

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere2d.rs')"></span>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso2::new(Vec2::y(), na::zero()); // 1.0 along the `y` axis.
let cylinder_pos = na::one::<Iso2<f32>>();           // Identity matrix.

/*
 * Compute their bounding spheres.
 */
let bounding_sphere_cone     = bounding_volume::bounding_sphere(&cone, &cone_pos);
let bounding_sphere_cylinder = bounding_volume::bounding_sphere(&cylinder, &cylinder_pos);

// Merge the two spheres.
let bounding_bounding_sphere = bounding_sphere_cone.merged(&bounding_sphere_cylinder);

// Enlarge the cylinder bounding sphere.
let loose_bounding_sphere_cylinder = bounding_sphere_cylinder.loosened(1.0);

// Intersection and inclusion tests.
assert!(bounding_sphere_cone.intersects(&bounding_sphere_cylinder));
assert!(bounding_bounding_sphere.contains(&bounding_sphere_cone));
assert!(bounding_bounding_sphere.contains(&bounding_sphere_cylinder));
assert!(!bounding_sphere_cylinder.contains(&bounding_bounding_sphere));
assert!(!bounding_sphere_cone.contains(&bounding_bounding_sphere));
assert!(loose_bounding_sphere_cylinder.contains(&bounding_sphere_cylinder));
```

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere3d.rs')"></span>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso3::new(Vec3::z(), na::zero()); // 1.0 along the `z` axis.
let cylinder_pos = na::one::<Iso3<f32>>();           // Identity matrix.

/*
 * Compute their bounding spheres.
 */
let bounding_sphere_cone     = bounding_volume::bounding_sphere(&cone, &cone_pos);
let bounding_sphere_cylinder = bounding_volume::bounding_sphere(&cylinder, &cylinder_pos);

// Merge the two spheres.
let bounding_bounding_sphere = bounding_sphere_cone.merged(&bounding_sphere_cylinder);

// Enlarge the cylinder bounding sphere.
let loose_bounding_sphere_cylinder = bounding_sphere_cylinder.loosened(1.0);

// Intersection and inclusion tests.
assert!(bounding_sphere_cone.intersects(&bounding_sphere_cylinder));
assert!(bounding_bounding_sphere.contains(&bounding_sphere_cone));
assert!(bounding_bounding_sphere.contains(&bounding_sphere_cylinder));
assert!(!bounding_sphere_cylinder.contains(&bounding_bounding_sphere));
assert!(!bounding_sphere_cone.contains(&bounding_bounding_sphere));
assert!(loose_bounding_sphere_cylinder.contains(&bounding_sphere_cylinder));
```
