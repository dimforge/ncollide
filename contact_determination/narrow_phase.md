# Narrow phase

A collision detector meant to be used during the broad phase must implement one
of two traits. First, the `narrow::CollisionDetector` trait is for contact
determination algorithms between two shapes with a type known at compile-time:

| Method | Description |
|--      | --          |
| `.update(...)`  | Given the two shapes and their position, determines their contact geometry without returning them. |
| `.num_colls() ` | The number of contacts detected by the last `.update(...)`.  |
| `.colls(out)`   | Collects to `out` the contacts detected by the last `.update(...)` |

A typical use of this kind of detector is to first `.update(...)` it and then collect its
result with a vector with `.colls(...)`.


However, a collision detector that works with a composite geometry like e.g.
the `Compound`, has to be able to perform himself some sort of mini-broad phase
so to find collision pairs between its individual parts. Those individual parts
having an exact type unknown at runtime, it is necessary to perform some sort
of redispatch to find the correct contact determination sub-algorithm.  This is
allowed by the `narrow::GeomGeomCollisionDetector`.

| Method | Description |
|--      | --          |
| `.update(dispatch, ...)`  | Given the two shapes and their position, determines their contact geometry without returning them. If a collision algorithm redispatch is necessary, use the first argument `dispatch`. |
| `.num_colls() ` | The number of contacts detected by the last `.update(...)`.  |
| `.colls(out)`   | Collects to `out` the contacts detected by the last `.update(...)` |

It is the same as the previous trait, except that the `.update(...)` method
takes two trait-objects as geometries and an instance of the
`narrow::GeomGeomDispatcher` that is able to find the correct collision
detection algorithm for those trait-objects.

Note that if you are not interested by the whole collision detection pipeline
but only by the exact contact determination algorithms, the
`narrow::GeomGeomDispatcher` can be used alone to retrieve the correct
collision detection algorithm, depending on your geometries:

```rust
let dispatcher = GeomGeom::Dispatcher::new(0.10);
let geom1 = Ball::new(0.5);
let geom2 = Cylinder::new(0.5, 1.0);
let geom3 = Cone::new(0.5, 1.0);

let ball_vs_cylinder_detector = dispatcher.dispatch(&geom1, &geom2);
let ball_vs_cone_detector     = dispatcher.dispatch(&geom1, &geom3);
let cylinder_vs_cone_detector = dispatcher.dispatch(&geom2, &geom3);
```

The following table shows which structure to use for which kind of object
pair:

|                     | Ball | `Implicit` geom | Mesh | Compound | Plane |
| --                  | :--: | :--: | :--: | :--: | :--: |
| **Ball**            | `BallBall` | `ImplicitImplicit` | `GeomConcaveGeom` | `GeomConcaveGeom` | `ImplicitPlane`    |
| **`Implicit` geom** | `ImplicitImplicit` | `ImplicitImplicit` | `GeomConcaveGeom` | `GeomConcaveGeom` | `ImplicitPlane` |
| **Mesh**            | `ConcaveGeomGeom` | `ConcaveGeomGeom` | $$\emptyset$$ | `ConcaveGeomGeom` | `ConcaveGeomGeom` |
| **Compound**        | `ConcaveGeomgeom` | `ConcaveGeomGeom` | `ConcaveGeomGeom` | `ConcaveGeomGeom` | `ConcaveGeomGeom` |
| **Plane**           | `PlaneImplicit` | `PlaneImplicit` | `GeomConcaveGeom` | `GeomConcaveGeom` | $$\emptyset$$ |

where _`Implicit` geom_ designs the `Capsule`, `Cone`, `Convex`, and `Cuboid`.
See the
[documentation](../doc/ncollide3df32/narrow/struct.GeomGeomDispatcher.html) of
the `GeomGeomDispatcher` for details about how to replace a contact
determination algorithm by your own.


Note that the `::new(...)` constructors of all those collision detectors take
an argument called the _prediction margin_. This will make them generate
contacts if the two objects are closer than this margin but not necessarily
touching. In this case the `depth` field of the resulting `Contact` is
negative and corresponds to the minimal distance separating them.  This is
useful to estimate how and when two objects that are _close enough_ may
eventually touch or penetrate if they are moved.

## Conforming contacts
All the algorithms presented above are limited to punctual contacts. Therefore
if you have a cube lying on a plane only one contact will be returned (right)
instead of a theoretically infinite set of point (left):

<center>
![](../img/conforming_contact.svg)
</center>


Those planar contacts that cannot be assimilated to a single point are called
_conforming contacts_. Approximating the shape of such contact with more than
one point is critical for some application like e.g. physics simulation. The
following figure shows what would happen with a conforming contact approximated
by only one point:

<center>
![](../img/contact.svg)
</center>

Here, the cube is falling toward the plane. When a contact is detected, the
cube is penetrating the plane and the physics engine will try to correct this
situation by applying a force to the contact point. This forces the object to
(unrealistically) rotate, moving the contact point to the over side.  This
alternation between two contact points due to unwanted rotations makes the
simulation instable and unrealistic. **ncollide** provides contact
determination algorithms wrappers that can generate a full contact manifold
either incrementally or in a single shot.

##### Incremental contact manifold generation
The `narrow::IncrementalContactManifoldGenerator` stores the results of the
wrapped collision detector, and updates them as the objects involved in the
contact move. After a few updates, a full manifold will be created:

<center>
![](../img/acc_contact.svg)
</center>

Obviously, storing the contacts points at each update indefinitely is costly
and not necessary. This is why if more than 2 (resp. 4) contacts in 2D (resp.
3D) are stored, the surplus will be removed. This removal is smart enough to
maximize the size of the area spanned by the remaining contact points. For
example, the red contact has been removed from the last part of the previous
figure because it is less significant than the two others.

The `IncrementalContactManifoldGenerator` can be used to wrap any structure
that implements the `CollisionDetector` trait and that generates a single
contact point:
```rust
let ball_vs_cube:  ImplicitImplicit<Ball, Cuboid> = ImplicitImplicit::new(0.04);
let plane_vs_ball: PlaneImplicit<Plane, Ball>     = PlaneImplicit::new(0.04);

let full_manifold_ball_vs_cube  = IncrementalContactManifoldGenerator::new(0.04, ball_vs_cube);
let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(0.04, plane_vs_ball);
```

##### One-shot contact manifold generation
The `narrow::OneShotContactManifoldGenerator` is slower but more accurate than
the `IncrementalContactManifoldGenerator` as it is able to generate a full
manifold as soon as only one contact has been detected. This is done by
applying small artificial rotations to one of the object in contact, and
accumulating the new points generated by the wrapped collision detector.
Therefore, from the user point of view, a full manifold has been generated:
<center>
![](../img/one_shot_contact.svg)
</center>

To reduce the computation times of this wrapper, the
`OneShotContactManifoldGenerator` stops this rotation-based algorithm and acts
like the incremental one as long as the manifold has been generated once.

The `OneShotContactManifoldGenerator` can be used to wrap any structure
that implements the `CollisionDetector` trait and that generates a single
contact point:
```rust
let ball_vs_cube:  ImplicitImplicit<Ball, Cuboid> = ImplicitImplicit::new(0.04);
let plane_vs_ball: PlaneImplicit<Plane, Ball>     = PlaneImplicit::new(0.04);

let full_manifold_ball_vs_cube  = OneShotContactManifoldGenerator::new(0.04, ball_vs_cube);
let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(0.04, plane_vs_ball);
```
