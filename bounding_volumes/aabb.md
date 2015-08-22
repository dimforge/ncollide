# Axis Aligned Bounding Box

As suggested by its name, the `bounding_volume::AABB` is a box with principal
axis aligned with the positive coordinate axis $$\bf x$$, $$\bf y$$, $$\bf z$$.

<center>
![aabb](../img/bounding_volume_aabb.svg)
</center>

Its orientation being fixed at all times, it is completely defined by the
position of its extremal vertices (the two vertices with extremal values along
each coordinate axis):

| Method   | Description                                                    |
|--        | --                                                             |
| `.mins()` | The AABB vertex with the smallest coordinates along each axis. |
| `.maxs()` | The AABB vertex with the greatest coordinates along each axis. |


Of course, the AABB implements the `BoundingVolume` trait. The
following shows the effect of the `.loosen(m)` method on it:

<center>
![loose aabb](../img/bounding_volume_aabb_loose.svg)
</center>

Finally, note that an AABB supports ray casting and point queries as described
by the [RayCast](../ray_casting/index.html) and
[PointQuery](../point_query/index.html) traits.

## Creating an AABB
There are four ways to create an AABB. The main one is to use the usual
static method `AABB::new(mins, maxs)`. This will fail if one component of
`mins` is strictly greater than the corresponding component of `maxs`. The
second one is to use the unsafe constructor `AABB::new_invalid()`. It is unsafe
because the result AABB is invalid: its `mins` field is set to
[Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html) and
its `maxs` field is set to
[-Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html).
This is useful to initiate the merging of multiple AABB.

The third is to use the `bounding_volume.aabb(g, m)` function, where `g` and
`m` are the shape and its position (e.g. a transformation matrix).

Finally, while this is not recommended except for generic programming, you may
as well directly call the method from the `bounding_volume::HasBoundingVolume`
trait implemented by any shape of `ncollide`:

| Method                | Description                                                |
|--                     | --                                                         |
| `.bounding_volume(m)` | Computes the aabb of `self` transformed by `m`. |

While using the trait method directly works (this is actually what
`bounding_volume.aabb(...)` does under the hood), the compiler might
sometimes fail to infer correctly the types involved in the trait
implementation and output a cryptic error message. Also note that the
`HasBoundingVolume` trait actually takes the bounding volume type as a type
parameter. Therefore, you may have to specify explicitly the return type of
the method in order to use it, e.g. `let bs: AABB<Pnt3<f32>> =
g.bounding_volume(m);`.

## Example

The following example computes the AABB of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder AABB, and
performs some tests.

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb2d.rs')"></span>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso2::new(Vec2::y(), na::zero()); // 1.0 along the `y` axis.
let cylinder_pos = na::one::<Iso2<f32>>();           // Identity matrix.

/*
 * Compute their axis-aligned bounding boxes.
 */
let aabb_cone     = bounding_volume::aabb(&cone, &cone_pos);
let aabb_cylinder = bounding_volume::aabb(&cylinder, &cylinder_pos);

// Merge the two boxes.
let bounding_aabb = aabb_cone.merged(&aabb_cylinder);

// Enlarge the cylinder aabb.
let loose_aabb_cylinder = aabb_cylinder.loosened(1.0);

// Intersection and inclusion tests.
assert!(aabb_cone.intersects(&aabb_cylinder));
assert!(bounding_aabb.contains(&aabb_cone));
assert!(bounding_aabb.contains(&aabb_cylinder));
assert!(!aabb_cylinder.contains(&bounding_aabb));
assert!(!aabb_cone.contains(&bounding_aabb));
assert!(loose_aabb_cylinder.contains(&aabb_cylinder));
```

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb3d.rs')"></span>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso3::new(Vec3::z(), na::zero()); // 1.0 along the `z` axis.
let cylinder_pos = na::one::<Iso3<f32>>();           // Identity matrix.

/*
 * Compute their axis-aligned bounding boxes.
 */
let aabb_cone     = bounding_volume::aabb(&cone, &cone_pos);
let aabb_cylinder = bounding_volume::aabb(&cylinder, &cylinder_pos);

// Merge the two boxes.
let bounding_aabb = aabb_cone.merged(&aabb_cylinder);

// Enlarge the cylinder aabb.
let loose_aabb_cylinder = aabb_cylinder.loosened(1.0);

// Intersection and inclusion tests.
assert!(aabb_cone.intersects(&aabb_cylinder));
assert!(bounding_aabb.contains(&aabb_cone));
assert!(bounding_aabb.contains(&aabb_cylinder));
assert!(!aabb_cylinder.contains(&bounding_aabb));
assert!(!aabb_cone.contains(&bounding_aabb));
assert!(loose_aabb_cylinder.contains(&aabb_cylinder));
```
