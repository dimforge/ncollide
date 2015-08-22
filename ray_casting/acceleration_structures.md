## Acceleration structures

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

## The BVT
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

#### Creating a BVT
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

#### Using a BVT

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

## Example

The following example creates four shapes, sets up a `BVT` to associate indices
to their bounding spheres, and casts some rays on it using the
`RayInterferencesCollector` visitor.

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt2d.rs')"></span>
```rust
/*
 * Custom trait to group HasBoudingSphere and RayCast together.
 */
trait Shape2 : HasBoundingSphere<Pnt2<f64>, Iso2<f64>> +
               RayCast<Pnt2<f64>, Iso2<f64>> {
}

impl<T> Shape2 for T
    where T: HasBoundingSphere<Pnt2<f64>, Iso2<f64>> +
             RayCast<Pnt2<f64>, Iso2<f64>> {
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vec2::new(1.0, 0.5));

    let shapes = [
        &ball as &Shape2,
        &caps as &Shape2,
        &cone as &Shape2,
        &cube as &Shape2
    ];

    let poss = [
        Iso2::new(Vec2::new(1.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(2.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(3.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(4.0, 2.0), na::zero())
    ];

    let idx_and_bounding_spheres  = vec!(
        (0usize, shapes[0].bounding_sphere(&poss[0])),
        (1usize, shapes[1].bounding_sphere(&poss[1])),
        (2usize, shapes[2].bounding_sphere(&poss[2])),
        (3usize, shapes[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vec2::x());
    let ray_miss = Ray::new(na::orig(), -Vec2::x());

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

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ray_bvt3d.rs')"></span>
```rust
/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape3: HasBoundingSphere<Pnt3<f64>, Iso3<f64>> +
              RayCast<Pnt3<f64>, Iso3<f64>> {
}

impl<T> Shape3 for T
    where T: HasBoundingSphere<Pnt3<f64>, Iso3<f64>> +
             RayCast<Pnt3<f64>, Iso3<f64>> {
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vec3::new(1.0, 0.5, 1.0));

    let shapes = [
        &ball as &Shape3,
        &caps as &Shape3,
        &cone as &Shape3,
        &cube as &Shape3
    ];

    let poss = [
        Iso3::new(Vec3::new(0.0, 0.0, 1.0), na::zero()),
        Iso3::new(Vec3::new(0.0, 0.0, 2.0), na::zero()),
        Iso3::new(Vec3::new(0.0, 0.0, 3.0), na::zero()),
        Iso3::new(Vec3::new(0.0, 2.0, 4.0), na::zero())
    ];

    let idx_and_bounding_spheres  = vec!(
        (0usize, shapes[0].bounding_sphere(&poss[0])),
        (1usize, shapes[1].bounding_sphere(&poss[1])),
        (2usize, shapes[2].bounding_sphere(&poss[2])),
        (3usize, shapes[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vec3::z());
    let ray_miss = Ray::new(na::orig(), -Vec3::z());

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
