# Bounding volumes

Performing some tests on an approximation of the shape of an object is often
useful to accelerate several geometric queries. For example testing two convex
polyhedrons for intersection is extremely time-consuming. Instead, we could
test that their spherical approximations (namely, their [bounding
spheres](#bounding-sphere)) intersect. Then if the approximations fail this
test there is no need to perform the same query on the original polyhedra. This
test-on-the-approximations-first approach is known as _pruning_.


The approximations presented here are conservative with regard to the object
volume, i.e., the approximated shape is completely contained inside of the
approximating object. Those are called _bounding volumes_. Many bounding
volumes exist on the literature, depending on their specific uses. For example,
the following figure shows a 2D polygon bounded by a bounding sphere, an Axis
Aligned Bounding Box (AABB), an Oriented Bounding Box (OBB), and a convex hull.
Not all of them are implemented on **ncollide** yet:

<center>
![bounding volumes](../img/bounding_volumes.svg)
</center>

Note that bounding volumes are very different from regular shapes: their
positions and orientations are completely encoded in the bounding volume
structure so they do not require a separate transformation matrix to reach any
position in space. All bounding volume must implement the `BoundingVolume`
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
is useful, e.g., to optimize some [broad
phase](../collision_detection_pipeline/#broad-phase) algorithms.


Finally, the `HasBoundingVolume` trait which is parametrized by the type of the
returned bounding volume is implemented by shapes and other entities that can
construct their own bounding volume given a transformation matrix:

| Method               | Description |
| --                   | --          |
| `.bounding_volume(m)` | Computes the bounding volume of `self` transformed by `m`. |


## Bounding Sphere

The `BoundingSphere` is a sphere that contains completely the bounded shape.
While this is the less tight bounding volume, it has the benefit of being
invariant with regard to isometric transformations. Thus, translating and
rotating the bounded shape will not modify the radius of its bounding sphere.
Bounding spheres support [ray casting](../../geometric_queries/#ray-casting) and
[point queries](../geometric_queries/#point-projection).

<center>
![bounding sphere](../img/bounding_volume_bounding_sphere.svg)
</center>

It is fully defined by its center and its radius:

| Method      | Description                                                    |
|--           | --                                                             |
| `.center()` | The bounding sphere center. |
| `.radius()` | The bounding sphere radius. |


Of course, the bounding sphere implements the `BoundingVolume` trait. The
following shows the effect of the `.loosen(m)` and `.tighten(m)` methods on it:

<center>
![Bounding sphere loosening](../img/bounding_volume_bounding_sphere_loose.svg)
![Bounding sphere tightening](../img/bounding_volume_bounding_sphere_tight.svg)
</center>

There are three ways to create a bounding sphere. The two main ones are to use
the usual static method `BoundingSphere::new(center, radius)` or with the
`bounding_volume::bounding_sphere(g, m)` function, where `g` and `m` are the
shape and its position (e.g. a transformation matrix). In generic code, you
might as well use the `HasBoundingVolume` trait.

The following example computes the bounding spheres of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder bounding
sphere, and performs some tests.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#bsphere_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#bsphere_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/bounding_sphere2d.rs')"></div>

</ul>

<div class="tab-content" markdown="1">
  <div id="bsphere_2D" class="tab-pane in active">
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
  </div>
  <div id="bsphere_3D" class="tab-pane">
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
  </div>
</div>

## Axis-Aligned Bounding Box

As suggested by its name, the `AABB` is a box with principal axis aligned with
the positive coordinate axises $\mathbf{x}$, $\mathbf{y}$, $\mathbf{z}$.

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

An AABB supports [ray casting](../../geometric_queries/#ray-casting) and [point queries](../geometric_queries/#point-projection) as well.

There are four ways to create an AABB. The main one is to use the usual
static method `AABB::new(mins, maxs)`. This will fail if one component of
`mins` is strictly greater than the corresponding component of `maxs`. The
second one is to use the unsafe constructor `AABB::new_invalid()`. It is unsafe
because the result AABB is invalid: its `mins` field is set to
[Bounded::max_value()](http://rust-num.github.io/num/num/trait.Bounded.html) and
its `maxs` field is set to
[-Bounded::max_value()](http://rust-num.github.io/num/num/trait.Bounded.html).
This is useful to initiate the merging of multiple AABB. The third construction
method is to use the `bounding_volume.aabb(g, m)` function, where `g` and `m`
are the shape and its position (e.g. a transformation matrix). Finally, generic
applications may directly call the method from the `HasBoundingVolume` trait.

The following example computes the AABB of a cone and a cylinder,
merges them together, creates an enlarged version of the cylinder AABB, and
performs some tests.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#aabb_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#aabb_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/aabb2d.rs')"></div>

</ul>

<div class="tab-content" markdown="1">
  <div id="aabb_2D" class="tab-pane in active">
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
  </div>
  <div id="aabb_3D" class="tab-pane">
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
  </div>
</div>


# Spacial partitioning

Acceleration structures like spacial partitioning and bounding volume
hierarchies are generalizations of bounding volumes to more than one shape.
They are necessary to efficiently perform geometric queries on scenes with
hundreds on objects. Acceleration structures allow to filter out quickly the
majority of objects that would make the geometric query fail. For example,
without an efficient spacial partitioning structure, we would not be able to
ray-trace a complex scene with millions of triangles like this one (6,704,264
triangles) in just a few seconds:

<center>
![rungholt](../img/rungholt.png)
</center>

For a high-level interface you may use a [broad
phase](../collision_detection_pipeline/#broad-phase) algorithm. Under the hood,
they use accelerations structures from the `partitioning` module that may be
used directly instead. At the moment, **ncollide** has only one tree-based
structure: the Bounding Volume Tree, aka., `BVT`. The similar structure `DBVT`
is less efficient but modifiable after initialization.

## The Bounding Volume Tree
The Bounding Volume Tree is a proper binary tree containing shapes on its
leaves only. Any interior node contains a bounding volume that is required to
bound all the shapes on the leaves of the subtree it is root of.  For example,
the following figure depicts a set of 2D objects (brown), their AABB (red) and
the corresponding AABB Tree (one color per depth):

<center>
![BVT with AABB](../img/AABB_tree_BVT.svg)
</center>

Note that even if this example uses AABB, the `BVT` and `DBVT` are generic with
regard to the type of bounding volume so we could use, e.g., bounding spheres
instead.

### Creating a BVT
Because the `BVT` is an immutable data structure, it must be created at once
and cannot be modified after. The `::new_with_partitioner(leaves, f)` is its
main constructor and requires a list `leaves` of tuples containing the objects
that will be stored on the BVT leaves and their bounding volumes. The objects
themselves are just associated data opaque to the `BVT` and do not have to
implement any specific trait. The second argument `f` is a closure (the
partitioning scheme) that will split any given array of bounding volumes into
two groups. This splitting process is known as the _top-down_ tree construction
approach, i.e., starting with the tree root and recursively splitting its way
down to the leaves. One example of such partitioning scheme is the
`partitioning::balanced_partitioner(...)` that will distribute the objects
depending on their bounding volumes position along one axis. This will generate
a balanced tree (with is not necessarily optimal for all applications).

The second constructor of the `BVT` is `::new_balanced(...)` which simply
invokes `::new_with_partitioner(...)` with your objects and the
`::balanced_partitioner(...)`.

### Using a BVT

A `BVT` can be traversed using the [visitor
pattern](https://en.wikipedia.org/wiki/Visitor_pattern). Three kinds of
traversals are available depending on your needs:

1. **Depth-first traversal** with `.visit(...)` controlled by a user-defined
   visitor implementing the `BVTVisitor` trait. An example of application of
   depth-first traversal is the search for all nodes intersecting a given
   bounding volume.
2. **Best-first traversal** with `.best_fisrt_search(...)` controlled by a
   user-defined cost function implementing the `BVTCostFn` trait. An example of
   application of best-first traversals is ray-tracing where you are only
   interested in the closest ray intersection. Best-first traversals are
   usually much more efficient than a complete traversal if only one result is
   needed.
3. **Simultaneous depth-first traversal** of two BVTs with `.visit_bvtt(...)`.
   This will traverse two BVT simultaneously, applying a user-defined visitor
   implementing the `BVTTVisitor` trait on each pair of nodes (one from each
   BVT) traversed. The BVTT acronym stands for Bounding Volume Test Tree
   because such traversal can be visualized as a tree as well. Simultaneous BVT
   traversal is typically used to check two composite object for intersection.
   Note that both BVT involved in the traversal may be the same one.

A few visitors and cost functions are already implemented on **ncollide**:

* The `BoundingVolumeInterferencesCollector` will collect references to all
  objects which bounding volume intersects the one given as argument
  to the visitor's constructor:

```rust
let interferences = Vec::new();

{
    let visitor = RayInterferencesCollector::new(&bv, &mut interferences);
    bvt.visit(&mut visitor);
}

// Now `interferences` contains the list of all objects which
// bounding volume intersects `bv`.
```

* The `RayInterferencesCollector` will collect references to all objects which
  bounding volume intersects the ray given as argument to the visitor's
  constructor:

```rust
let result = Vec::new();

{
    let visitor = RayInterferencesCollector::new(&ray, &mut result);
    bvt.visit(&mut visitor);
}

// Now `result` contains the list of all objects which
// bounding volume intersects `ray`.
```

* The `PointInterferencesCollector` will collect references to all objects
  which bounding volume contains the point given as argument to the visitor's
  constructor:

```rust
let result = Vec::new();

{
    let visitor = PointInterferencesCollector::new(&point, &mut result);
    bvt.visit(&mut visitor);
}

// Now `result` contains the list of all objects which
// bounding volume intersects `ray`.
```

* The `RayIntersectionCostFn` will search for the closest object that
  intersects the ray given as argument to the visitor's constructor. The BVT
  user-data must implement the `RayCast` trait:

```rust
let visitor = RayIntersectionCostFn::new(&ray, true, false);

match bvt.best_first_search(&mut visitor) {
    Some((body, ray_intersection)) => {
        // The ray intersected some objects and `body` is the closest one.
        // `ray_intersection` contains the ray-cast result.
    },
    None => {
        // No intersection found.
    }
}
```

**Attention:** note that while the cost function `RayIntersectionCostFn`
performs a ray cast on both objects and their bounding volumes, the other
visitors like `RayInterferencesCollector` only work with the bounding volumes.
So if you are using the latter, you need to check if the query actually
succeeds on the collected objects yourself!


The following example creates four shapes, sets up a `BVT` to associate indices
to their bounding spheres, and casts some rays on it using the
`RayInterferencesCollector` visitor.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#bvt_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#bvt_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt2d.rs')"></div>

</ul>

<div class="tab-content" markdown="1">
  <div id="bvt_2D" class="tab-pane in active">
```rust
/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
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
  </div>
  <div id="bvt_3D" class="tab-pane">
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
  </div>
</div>

### The DBVT
The Dynamic Bounding Volume Tree shares the same overall structure as the `BVT`
but is modifiable after initialization. It allows:

* Insersion of a new object `b` with its bounding volume `bv` with
  `.insert_new(b, bv)`. This will return a leaf that may be manipulated later.
  This has a $\mathcal{O}(\log(n))$ average time complexity.
* Removal of a leaf from the tree with `.remove(leaf)`. This has a
  $\mathcal{O}(1)$ time complexity.
* Insertion of an unrooted leaf with `.insert(leaf)`.  This has a
  $\mathcal{O}(\log(n))$ average time complexity.  An unrooted leaf is one that
  has been removed from its tree. The same leaf may not be added to two trees
  simultaneously but it can be moved to another `DBVT` instance after being
  removed from the original one.

After an insertion or a removal, the `DBVT` must recompute some internal node
bounding volumes in order to ensure they still bound their subtree's leaves.
This refitting is performed immediately at insertion-time and lazily after
removals.

Currently, the only way to traverse the `DBVT` is with the `.visit(...)` method
which will perform a **depth-first traversal** using a user-defined visitor
implementing the `BVTVisitor` trait.

