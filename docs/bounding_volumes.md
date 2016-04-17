# Bounding volumes

Performing some tests on an approximation of the shape of an object is often
useful to fasten several geometric queries. For example testing two convex
polyhedrons for intersection is extremely time-consuming. Instead, we could
test that their spherical approximations (namely, their bounding spheres)
intersect; and if the approximations fail this test there is no need to perform
the same query on the original polyhedra. This test-on-the-approximations-first
approach is known as _prunning_.


The approximations presented here are conservative with regard to the object
volume, i.e., the approximated shape is completely contained inside of the
approximating object. This is called a bounding volume. Many bounding volumes
exist on the literature, depending on their specific uses. For example, the
following figure shows a 2D polygon bounded by a bounding sphere, an Axis
Aligned Bounding Box (AABB), an Oriented Bounding Box (OBB), and a convex hull.
Not all of them are implemented on **ncollide** yet:

<center>
![bounding volumes](../img/bounding_volumes.svg)
</center>

Note that bounding volumes are very different from regular shapes: their
position in space is completely contained by the bounding volume structure so
they do not require a separate transformation matrix to reach any position in
space. In addition, they must implement the `bounding_volume::BoundingVolume`
trait:


| Method            | Description |
|--                 | --          |
| `.intersects(bv)` | Checks `self` for intersection with `bv`.              |
| `.contains(bv)`   | Returns `true` if `bv` is completely inside of `self`. |
| `.merge(bv)`      | Merge `self` and `bv` in-place. |
| `.merged(bv)`     | Returns a bounding volume, result of the merge of `self` with `bv`. |
| `.loosen(m)`      | Dilates `self` by a ball of radius `m` in-place.          |
| `.loosened(m)`    | Returns a copy of `self` dilated by a ball of radius `m`. |
| `.tighten(m)`     | Erodes `self` by a ball of radius `m` in-place.          |
| `.tightened(m)`   | Returns a copy of `self` eroded by a ball of radius `m`. |

The `.loosen(...)` and `.loosened(...)` (resp. `.tighten(...)` and
`.tightened(...)`) methods allow you to dilate (resp. erode) the bounding
volume by a given margin. This will effectively make the new bounding volume
strictly larger (resp. thinner) than the original one if `m` is not zero.  This
is useful, e.g., to optimize some [broad phase](../contact_determination/broad_phase.html)
algorithms.


Finally, the `bounding_volume::HasBoundingVolume` trait which is parametrized
by the type of the returned bounding volume is implemented by shapes and other
entities that can construct their bounding volume, given a transformation
matrix:

| Method               | Description |
| --                   | --          |
| `.bounding_volume(m)` | Computes the bounding volume of `self` transformed by `m`. |


## Bounding Sphere

The `bounding_volume::BoundingSphere` is a sphere that contains completely the
bounded shape. While this is the less tight bounding volume, it has the benefit
of being invariant with regard to isometric transformations. Thus, translating
and rotating the bounded shape do not modify the radius of the bounding
sphere.

<center>
![bounding sphere](../img/bounding_volume_bounding_sphere.svg)
</center>

It is fully defined by its center and its radius:

| Method      | Description                                                    |
|--           | --                                                             |
| `.center()` | The bounding sphere center. |
| `.radius()` | The bounding sphere radius. |


Of course, the bounding sphere implements the `BoundingVolume` trait. The
following shows the effect of the `.loosen(m)` and `.tighten(m)` method on it:

<center>
![Bounding sphere loosening](../img/bounding_volume_bounding_sphere_loose.svg)
![Bounding sphere tightening](../img/bounding_volume_bounding_sphere_tight.svg)
</center>

Finally, note that a bounding sphere supports ray casting and point queries as
described by the [RayCast](../ray_casting/index.html) and
[PointQuery](../point_query/index.html) traits.

### Creating a Bounding Sphere

There are three ways to create a bounding sphere. The main one is to use the usual
static method `BoundingSphere::new(center, radius)`. The second is to use the
`bounding_volume.bounding_sphere(g, m)` function, where `g` and `m` are the
shape and its position (e.g. a transformation matrix).  While this is not
recommended except for generic programming, you may as well directly call the
method from the `bounding_volume::HasBoundingVolume` trait implemented by any
shape of `ncollide`:

| Method                | Description                                                |
|--                     | --                                                         |
| `.bounding_volume(m)` | Computes the bounding sphere of `self` transformed by `m`. |

While using the trait method directly works (this is actually what
`bounding_volume.bounding_sphere(...)` does under the hood), the compiler might
sometimes fail to infer correctly the types involved in the trait
implementation and output a cryptic error message. Also note that the
`HasBoundingVolume` trait actually takes the bounding volume type as a type
parameter. Therefore, you may have to specify explicitly the return type of
the method in order to use it, e.g. `let bs: BoundingSphere<Point3<f32>> =
g.bounding_volume(m);`.

### Example

The following example computes the bounding spheres of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder bounding
sphere, and performs some tests.

#### 2D example <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere2d.rs')"></div>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Isometry2::new(Vector2::y(), na::zero()); // 1.0 along the `y` axis.
let cylinder_pos = na::one::<Isometry2<f32>>();              // Identity matrix.

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

#### 3D example <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere3d.rs')"></div>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Isometry3::new(Vector3::z(), na::zero()); // 1.0 along the `z` axis.
let cylinder_pos = na::one::<Isometry3<f32>>();              // Identity matrix.

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

## Axis-Aligned Bounding Box

As suggested by its name, the `bounding_volume::AABB` is a box with principal
axis aligned with the positive coordinate axises $\mathbf{x}$, $\mathbf{y}$, $\mathbf{z}$.

<center>
![AABB](../img/bounding_volume_aabb.svg)
</center>

Its orientation being fixed at all times, it is completely defined by the
position of its extremal vertices (the two vertices with extremal values along
each coordinate axis):

| Method   | Description                                                    |
|--        | --                                                             |
| `.mins()` | The AABB vertex with the smallest coordinates along each axis. |
| `.maxs()` | The AABB vertex with the greatest coordinates along each axis. |


Of course, the AABB implements the `BoundingVolume` trait. The following shows
the effect of the `.loosen(m)` and `.tighten(m)` method on it:

<center>
![AABB loosening](../img/bounding_volume_aabb_loose.svg)
![AABB tightening](../img/bounding_volume_aabb_tight.svg)
</center>

Finally, note that an AABB supports ray casting and point queries as described
by the [RayCast](../ray_casting/index.html) and
[PointQuery](../point_query/index.html) traits.

### Creating an AABB
There are four ways to create an AABB. The main one is to use the usual
static method `AABB::new(mins, maxs)`. This will fail if one component of
`mins` is strictly greater than the corresponding component of `maxs`. The
second one is to use the unsafe constructor `AABB::new_invalid()`. It is unsafe
because the result AABB is invalid: its `mins` field is set to
[Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html) and
its `maxs` field is set to
[-Bounded::max_value()](http://doc.rust-lang.org/std/num/trait.Bounded.html).
This is useful to initiate the merging of multiple AABB. The third construction
method is to use the `bounding_volume.aabb(g, m)` function, where `g` and `m`
are the shape and its position (e.g. a transformation matrix).  Finally, while
this is not recommended except for generic programming, you may as well
directly call the method from the `bounding_volume::HasBoundingVolume` trait
implemented by any shape of `ncollide`:

| Method                | Description                                                |
|--                     | --                                                         |
| `.bounding_volume(m)` | Computes the aabb of `self` transformed by `m`. |

While using the trait method directly works (this is actually what
`bounding_volume.aabb(...)` does under the hood), the compiler might
sometimes fail to infer correctly the types involved in the trait
implementation and output a cryptic error message. Also note that the
`HasBoundingVolume` trait actually takes the bounding volume type as a type
parameter. Therefore, you may have to specify explicitly the return type of
the method in order to use it, e.g. `let bs: AABB<Point3<f32>> =
g.bounding_volume(m);`.

### Example

The following example computes the AABB of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder AABB, and
performs some tests.

#### 2D example <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb2d.rs')"></div>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Isometry2::new(Vector2::y(), na::zero()); // 1.0 along the `y` axis.
let cylinder_pos = na::one::<Isometry2<f32>>();              // Identity matrix.

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

#### 3D example <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb3d.rs')"></div>
```rust
/*
 * Initialize the shapes.
 */
let cone     = Cone::new(0.5, 0.5);
let cylinder = Cylinder::new(1.0, 0.5);

let cone_pos     = Isometry3::new(Vector3::z(), na::zero()); // 1.0 along the `z` axis.
let cylinder_pos = na::one::<Isometry3<f32>>();              // Identity matrix.

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


# Spacial partitioning

Without an efficient spacial partitioning structure, we would not be able to
ray-trace a complex scene with millions of triangles like this one (6,704,264
triangles):

<center>
![rungholt](../img/rungholt.png)
</center>

If your objects can move, you may use a [broad
phase](../contact_determination/broad_phase.html) since the `BroadPhase` trait
warrants ray-casting capabilities. Though a lighter and often simpler
alternative is to use a space-partitioning structure like the
`partitioning::DBVT` directly.

If your objects do not move, the `partitioning::BVT` will be more efficient and
usable on a multi-threaded context.

## The bounding volume tree
For a static scene, better performances will be achieved by an immutable
spacial partitioning structure. At the moment, the only one implemented by
**ncollide** is the Bounding Volume Tree `BVT`. For example, the following
figure depicts a set of 2D objects (brown), their AABB (black) and the
corresponding AABB Tree (one color per depth):

<center>
![BVT with AABB](../img/AABB_tree_BVT.svg)
</center>

Note that even if this example uses AABB, the `BVT` is generic with regard to
the type of bounding volume (we could use e.g. bounding spheres instead).

### Creating a BVT
Because the BVT is an immutable data structure, it must be created at once and
cannot be modified after. The `::new_with_partitioner(...)` is its main
constructor and requires a list of tuples containing the objects that will be
stored on the BVT leaves and their bounding volumes. The objects themselves do
not have to implement any specific trait. The other argument is a
partitioning scheme that will, given an array of bounding volumes, split them
into two groups. This splitting process is known as the _top-down_ tree
construction approach. One example of such partitioning scheme is the
`partitioning::balanced_partitionner(...)` that will distribute the objects
depending on their bounding volumes position along one axis relative to their
median. This will generate a balanced tree.

The second constructor of the BVT is `::new_balanced(...)` which simply invokes
`::new_with_partitioner(...)` with your objects and the
`::balanced_partitionner(...)`.

### Using a BVT

There are two ways of using the BVT for ray casting. The first one is using the
`partitioning::RayInterferencesCollector` visitor:

```rust
let bvt           = BVT::new_balanced(your_objects);
let intersections = Vec::new();

{
    let visitor = RayInterferencesCollector::new(&ray, &mut intersections);
    bvt.visit(&mut visitor);
}

// Now `intersections` contains the list of all objects which bounding volume intersects the ray.
```

If you are not interested in the complete set of objects that intersect the ray
but only the closest one, using the BVT `.best_first_search(...)` method:

```rust
let bvt = BVT::new_balanced(your_objects);

let visitor = RayIntersectionCostFn::new(&ray, true, false);
match bvt.best_first_search(&mut visitor) {
    Some((body, ray_intersection)) => {
        // The ray intersected some objects and `body` is the closest one.
    },
    None => {
        // No intersection found.
    }
}
```

The end-result is the objects which bounding volume has the smallest time of
impact with the ray and the related geometric informations of type
`RayIntersection`.


**Attention:** note that while the cost function `RayIntersectionCostFn` performs a
ray cast on both the objects and their bounding volumes, the visitor
`RayInterferencesCollector` only works with the bounding volume. So if you are
using the latter, you need to check if the ray actually intersects the objects
yourself!


The following example creates four shapes, sets up a `BVT` to associate indices
to their bounding spheres, and casts some rays on it using the
`RayInterferencesCollector` visitor.

#### 2D example <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt2d.rs')"></div>
```rust
/*
 * Custom trait to group HasBoudingSphere and RayCast together.
 */
trait Shape2: HasBoundingSphere<Point2<f64>, Isometry2<f64>> +
              RayCast<Point2<f64>, Isometry2<f64>> {
}

impl<T> Shape2 for T
    where T: HasBoundingSphere<Point2<f64>, Isometry2<f64>> +
             RayCast<Point2<f64>, Isometry2<f64>> {
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vector2::new(1.0, 0.5));

    let shapes = [
        &ball as &Shape2,
        &caps as &Shape2,
        &cone as &Shape2,
        &cube as &Shape2
    ];

    let poss = [
        Isometry2::new(Vector2::new(1.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(2.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(3.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(4.0, 2.0), na::zero())
    ];

    let idx_and_bounding_spheres  = vec!(
        (0usize, shapes[0].bounding_sphere(&poss[0])),
        (1usize, shapes[1].bounding_sphere(&poss[1])),
        (2usize, shapes[2].bounding_sphere(&poss[2])),
        (3usize, shapes[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vector2::x());
    let ray_miss = Ray::new(na::orig(), -Vector2::x());

    /*
     * Collecting all objects with bounding volumes intersecting the ray.
     */
    let mut collector_hit:  Vec<usize> = Vec::new();
    let mut collector_miss: Vec<usize> = Vec::new();

    // We need a new scope here to avoid borrowing issues.
    {
        let mut visitor_hit  = RayInterferencesCollector::new(&ray_hit, &mut collector_hit);
        let mut visitor_miss = RayInterferencesCollector::new(&ray_miss, &mut collector_miss);

        bvt.visit(&mut visitor_hit);
        bvt.visit(&mut visitor_miss);
    }

    assert!(collector_hit.len()  == 3);
    assert!(collector_miss.len() == 0);
}
```

#### 3D example <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt3d.rs')"></div>
```rust
/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape3: HasBoundingSphere<Point3<f64>, Isometry3<f64>> +
              RayCast<Point3<f64>, Isometry3<f64>> {
}

impl<T> Shape3 for T
    where T: HasBoundingSphere<Point3<f64>, Isometry3<f64>> +
             RayCast<Point3<f64>, Isometry3<f64>> {
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vector3::new(1.0, 0.5, 1.0));

    let shapes = [
        &ball as &Shape3,
        &caps as &Shape3,
        &cone as &Shape3,
        &cube as &Shape3
    ];

    let poss = [
        Isometry3::new(Vector3::new(0.0, 0.0, 1.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, 2.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, 3.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 2.0, 4.0), na::zero())
    ];

    let idx_and_bounding_spheres  = vec!(
        (0usize, shapes[0].bounding_sphere(&poss[0])),
        (1usize, shapes[1].bounding_sphere(&poss[1])),
        (2usize, shapes[2].bounding_sphere(&poss[2])),
        (3usize, shapes[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vector3::z());
    let ray_miss = Ray::new(na::orig(), -Vector3::z());

    /*
     * Ray cast using a visitor.
     */
    let mut collector_hit:  Vec<usize> = Vec::new();
    let mut collector_miss: Vec<usize> = Vec::new();

    // We need a new scope here to avoid borrowing issues.
    {
        let mut visitor_hit  = RayInterferencesCollector::new(&ray_hit, &mut collector_hit);
        let mut visitor_miss = RayInterferencesCollector::new(&ray_miss, &mut collector_miss);

        bvt.visit(&mut visitor_hit);
        bvt.visit(&mut visitor_miss);
    }

    assert!(collector_hit.len()  == 3);
    assert!(collector_miss.len() == 0);
}
```
