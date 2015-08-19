# Broad phase

A broad phase on **ncollide** must implement the `broad_phase::BroadPhase`
trait that ensures that objects can be added, removed, updated, and supports
common geometric queries:

| Method                 | Description                                     |
|--                      | --                                              |
| `.defered_add(uid, bv, data)`                     | Informs the broad phase algorithm that a new object with the identifier `uid`, the bounding volume `bv`, and the associated data `data` has to be added during the next update. |
| `.defered_remove(object)`                         | Informs the broad phase algorithm that the object identified by `uid` must be removed at the next update. |
| `.defered_set_bounding_volume(uid, bv)`           | Informs the broad phase algorithm that the object identified by `uid`’s bounding volume has to be replaced by `bv` at the next update. |
| `.update(filter, callback)`                       | Updates this broad phase algorithm, actually performing object addition and removal. `filter` is a predicate that indicates if a new potential collision pair is valid. If it is (`filter` returns `true`), `callback` will be called for each such now collision pair. `callback` is also called |
| `.interferences_with_bounding_volume(bv, result)` | Fills `result` with references to each object which bounding volume intersects the bounding volume `bv`. |
| `.interferences_with_ray(ray, result)`            | Fills `result` with references to each object which bounding volume intersects the ray `ray`. |
| `.interferences_with_point(point, result)`        | Fills `result` with references to each object which bounding volume contains the point `point`. |

Let us clarify what _associated data_ means here. A broad phase is guaranteed
to associate some pieces of data to each object. Those data are completely
user-defined (e.g. they can even be as general as `Box<Any>`) and are passed as
argument to the user-defined callbacks when the `update` method is called.
Therefore, feel free to store in there any piece of data that may be useful to
identify the object on your side and to filter out unwanted collision pairs.

### The DBVT broad phase

The `broad_phase::DBVTBroadPhase` is based on a Dynamic Bounding Volume Tree (DBVT)
to detect interferences. It implements all four broad phase-related traits
described above.


The `partitioning::DBVT` structure itself is a proper binary tree that maps a
bounding volume to the object it bounds on its leaves. The internal nodes only
contain a bounding volume that bounds all its children. The following figure
shows an example of tree that bounds a set of brown objects with their red
AABB. Note that, instead of the exact bounding volumes (read), the
`DBVTBroadPhase` stores their loosened version (black):

<center>
![dbvt](../img/AABB_tree_DBVT.svg)
</center>

Creating a `DBVTBroadPhase` is simple using the idiomatically named
`::new(margin, small_keys)` function:
```rust
let mut dbvt = DBVTBroadPhase::new(0.08, false);
```
Storing the loosened bounding volumes instead of the exact ones is a
significant optimization for scenes where the broad phase has to track contact
pairs involving slow-moving objects: the DBVT is updated only when the
displacement of an object is large enough to make its exact bounding volume
move out of the loosened version stored on the tree. That way objects moving at
high frequency but low amplitude will almost never trigger an update, at the
cost of a less tight bounding volume for interference detection (i.e. more
false positives). The amount of loosening is controlled by the first argument
`margin`. The second argument `small_keys` is here for optimization
purpose. Set it to `true` if and only if you know that the integer keys you use
to identify your objects are small enough (as in "small enough for them to be
used as keys on a `Vec` instead of a `HashMap`"). If you are not sure of the
values your keys may take, set `small_keys` to `false`.


## Example
The following example creates four balls, adds them to a `DBVTBroadPhase`,
updates the broad phase, and removes some of them.

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/dbvt_broad_phase2d.rs')" ></span>
```rust
/*
 * Create the objects.
 */
let poss = [ Iso2::new(Vec2::new(0.0, 0.0), na::zero()),
             Iso2::new(Vec2::new(0.0, 0.5), na::zero()),
             Iso2::new(Vec2::new(0.5, 0.0), na::zero()),
             Iso2::new(Vec2::new(0.5, 0.5), na::zero()) ];

// We will use the same geometry for the four objects.
let ball = Ball::new(0.5);

/*
 * Create the broad phase.
 * We know we will use small uids (from 0 to 3)so we can pass `true` as the second argument.
 */
let mut bf = DBVTBroadPhase::new(0.2, true);

// First parameter:  a unique id for each object.
// Second parameter: the object bounding box.
// Third parameter:  some data (here, the id that identify each object).
bf.defered_add(0, bounding_volume::aabb(&ball, &poss[0]), 0);
bf.defered_add(1, bounding_volume::aabb(&ball, &poss[1]), 1);
bf.defered_add(2, bounding_volume::aabb(&ball, &poss[2]), 2);
bf.defered_add(3, bounding_volume::aabb(&ball, &poss[3]), 3);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| { });

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.defered_remove(0);
bf.defered_remove(1);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a ,b| *a != *b, &mut |_, _, _| { });

assert!(bf.num_interferences() == 1)
```

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/dbvt_broad_phase3d.rs')" ></span>
```rust
/*
 * Create the objects.
 */
let poss = [ Iso3::new(Vec3::new(0.0, 0.0, 0.0), na::zero()),
             Iso3::new(Vec3::new(0.0, 0.5, 0.0), na::zero()),
             Iso3::new(Vec3::new(0.5, 0.0, 0.0), na::zero()),
             Iso3::new(Vec3::new(0.5, 0.5, 0.0), na::zero()) ];

// We will use the same geometry for the four objects.
let ball = Ball::new(0.5);

/*
 * Create the broad phase.
 * We know we will use small uids (from 0 to 3)so we can pass `true` as the second argument.
 */
let mut bf = DBVTBroadPhase::new(0.2, true);

// First parameter:  a unique id for each object.
// Second parameter: the object bounding box.
// Third parameter:  some data (here, the id that identify each object).
bf.defered_add(0, bounding_volume::aabb(&ball, &poss[0]), 0);
bf.defered_add(1, bounding_volume::aabb(&ball, &poss[1]), 1);
bf.defered_add(2, bounding_volume::aabb(&ball, &poss[2]), 2);
bf.defered_add(3, bounding_volume::aabb(&ball, &poss[3]), 3);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| { });

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.defered_remove(0);
bf.defered_remove(1);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a ,b| *a != *b, &mut |_, _, _| { });

assert!(bf.num_interferences() == 1)
```
