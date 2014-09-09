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
| `mins()` | The AABB vertex with the smallest coordinates along each axis. |
| `maxs()` | The AABB vertex with the greatest coordinates along each axis. |


The AABB implements the `LooseBoundingVolume` trait so it can be enlarged by an
arbitrary margin $$m$$:

<center>
![loose aabb](../img/bounding_volume_aabb_loose.svg)
</center>

Finally, note that an AABB supports ray casting as described by the
[RayCast](../ray_casting/README.html) trait.

## Creating an AABB
There are three ways to create an AABB. The main one is to use the usual
static method `AABB::new(mins, maxs)`. This will fail if one component of
`mins` is strictly greater than the corresponding component of `maxs`. The
second one is to use the unsafe constructor `AABB::new_invalid()`. It is unsafe
because the result AABB is invalid: its `mins` field is set to
[Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html) and
its `maxs` field is set to
[-Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html).
This is useful to initiate the merging of multiple AABB.


The last method to create an AABB is using the `boundoing_volume::HasAABB`
trait implemented by any `Geom`:

| Method    | Description                                     |
|--         | --                                              |
| `aabb(m)` | Computes the AABB of `self` transformed by `m`. |

This is the simplest way to compute the AABB of a geometry defined by
**ncollide**. Do not forget to explicitly import the trait in order to be
allowed call this method: `use ncollide::bounding_volume::HasAABB`!

## Working example

The following examples compute the AABB of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder AABB, and
performs some tests.

###### 2D example <div class="d2" onclick="window.open('../src/aabb2d.rs')"></div>
```rust
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso2::new(Vec2::y(), na::zero());
let cylinder_pos = na::one();

let aabb_cone     = cone.aabb(&cone_pos);
let aabb_cylinder = cylinder.aabb(&cylinder_pos);

let bounding_aabb = aabb_cone.merged(&aabb_cylinder);

let loose_aabb_cylinder = aabb_cylinder.loosened(1.0);

assert!(aabb_cone.intersects(&aabb_cylinder));
assert!(bounding_aabb.contains(&aabb_cone));
assert!(bounding_aabb.contains(&aabb_cylinder));
assert!(!aabb_cylinder.contains(&bounding_aabb));
assert!(!aabb_cone.contains(&bounding_aabb));
assert!(loose_aabb_cylinder.contains(&aabb_cylinder));
```

###### 3D example <div class="d3" onclick="window.open('../src/aabb3d.rs')"></div>
```rust
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Iso3::new(Vec3::z(), na::zero());
let cylinder_pos = na::one();

let aabb_cone     = cone.aabb(&cone_pos);
let aabb_cylinder = cylinder.aabb(&cylinder_pos);

let bounding_aabb = aabb_cone.merged(&aabb_cylinder);

let loose_aabb_cylinder = aabb_cylinder.loosened(1.0);

assert!(aabb_cone.intersects(&aabb_cylinder));
assert!(bounding_aabb.contains(&aabb_cone));
assert!(bounding_aabb.contains(&aabb_cylinder));
assert!(!aabb_cylinder.contains(&bounding_aabb));
assert!(!aabb_cone.contains(&bounding_aabb));
assert!(loose_aabb_cylinder.contains(&aabb_cylinder));
```
