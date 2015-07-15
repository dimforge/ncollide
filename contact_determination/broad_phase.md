# Broad phase

A broad phase on **ncollide** must implement the `broad_phase::BroadPhase`
trait that ensures that objects can be added, removed, updated, and supports
common geometric queries:

| Method                 | Description                                     |
|--                      | --                                              |
| `.add(object)`         | Adds `object` to this broad phase algorithm.      |
| `.remove(object)`      | Removes `object` from this broad phase algorithm. |
| `.update()`            | Updates this broad phase algorithm.             |
| `.update_object(object)` | Partially updates this broad phase algohithm so that all the pairs involving `object` are detected. |
| `.interferences_with_bounding_volume(bv, result)` | Clones to `result` any object that intersects the bounding volume `bv`. |
| `.interferences_with_ray(ray, result)` | Clones to `result` any object that intersects the bounding volume `bv`. |
| `.interferences_with_point(point, result)` | Clones to `result` any object that contains the point `point`. |
| `.for_each_pair(f)`     | Applies the closure `f` to each contact pair and its _associated data_. |
| `.for_each_pair_mut(f)` | Applies the closure `f` to each contact pair and a mutable reference to its _associated data_. 
| `.deactivate(object)`   | Deactivates `object`. Two deactivated objects cannot be in contact. |
| `.activate(object, f)`  | Activates `object` and applies the closure `f` on each new contact pairs involving the activated object. | 

Let us clarify what _associated data_ means here. A broad phase must associate
some data to each collision pair. Usually, this data is a collision detector
algorithm that will be used during the [Narrow
Phase](../contact_determination/narrow_phase.html). The method used by the
broad phase to generate this piece of data for each potential collision pair is
implementation-specific but most of the time it will use a factory that
implements the `broad_phase::Dispatcher` trait:

| Method                   | Description                                     |
|--                        | --                                              |
| `.dispatch(object1, object2)` | Instantiates the data associated to the potential contact pair involving `object1` and `object2`. |
| `.is_valid(object1, object2)` | Tests if a collision pair between `object1` and `object2` is valid. |

Note that the life lengths of those _associated data_ is also
implementation-dependent. Therefore, if you write generic code that do not know
the exact type of the broad phase, you should not rely on the destruction time
(i.e. call to the `drop` method) of the data to perform useful tasks.

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

Creating a `DBVTBroadPhase` is simple as long as you have a structure that
implements the `Dispatcher` trait described above. You must also give the
desired margin the bounding volumes will be loosened by.

```rust
let dispatcher = NoIdDispatcher::new(); // No self-interference.
let mut dbvt   = DBVTBroadPhase::new(dispatcher, 0.08);
```
Storing the loosened bounding volumes instead of the exact ones is a
significant optimization for scenes where the broad phase has to track contact
pairs involving slow-moving objects: the DBVT is updated only when the
displacement of an object is large enough to make its exact bounding volume
move out of the loosened version stored on the tree. That way objects moving at
high frequency but low amplitude will almost never trigger an update, at the
cost of a less tight bounding volume for interference detection (i.e. more
false positives).

## Example
The following example creates four balls, adds them to a `DBVTBroadPhase`,
deactivates, reactivates, and removes them. The `bounding_volume::WithAABB`
structure associates a position to a shape that implement the `HasAABB` trait.

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
