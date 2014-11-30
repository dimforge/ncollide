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

###### 2D example <span class="d2" onclick="window.open('../src/dbvt_broad_phase2d.rs')" ></span>
```rust
type Shape<'a> = Rc<RefCell<WithAABB<Ball>>>;

/*
 * Create the objects.
 */
let pos1 = Iso2::new(Vec2::new(0.0, 0.0), na::zero());
let pos2 = Iso2::new(Vec2::new(0.0, 0.5), na::zero());
let pos3 = Iso2::new(Vec2::new(0.5, 0.0), na::zero());
let pos4 = Iso2::new(Vec2::new(0.5, 0.5), na::zero());

let shape1 = Ball::new(0.5);
let shape2 = Ball::new(0.5);
let shape3 = Ball::new(0.5);
let shape4 = Ball::new(0.5);

let obj1 = Rc::new(RefCell::new(WithAABB(pos1, shape1)));
let obj2 = Rc::new(RefCell::new(WithAABB(pos2, shape2)));
let obj3 = Rc::new(RefCell::new(WithAABB(pos3, shape3)));
let obj4 = Rc::new(RefCell::new(WithAABB(pos4, shape4)));

/*
 * Create the broad phase.
 */
let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher::new();
let mut bf = DBVTBroadPhase::new(dispatcher, 0.2);

bf.add(obj1.clone());
bf.add(obj2.clone());
bf.add(obj3.clone());
bf.add(obj4.clone());

bf.update();

assert!(bf.num_interferences() == 6);

// Deactivate everybody.
bf.deactivate(&obj1);
bf.deactivate(&obj2);
bf.deactivate(&obj3);
bf.deactivate(&obj4);

// Deactivated bodies do not interfere with each other.
assert_eq!(bf.num_interferences(), 0);

// Reactivate everybody.
bf.activate(&obj1, |_, _, _| { });
bf.activate(&obj2, |_, _, _| { });
bf.activate(&obj3, |_, _, _| { });
bf.activate(&obj4, |_, _, _| { });

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.remove(&obj1);
bf.remove(&obj2);

// Update the broad phase.
bf.update();

assert!(bf.num_interferences() == 1)
```

###### 3D example <span class="d3" onclick="window.open('../src/dbvt_broad_phase3d.rs')" ></span>
```rust
type Shape<'a> = Rc<RefCell<WithAABB<Ball>>>;

/*
 * Create the objects.
 */
let pos1 = Iso3::new(Vec3::new(0.0, 0.0, 0.0), na::zero());
let pos2 = Iso3::new(Vec3::new(0.0, 0.5, 0.0), na::zero());
let pos3 = Iso3::new(Vec3::new(0.5, 0.0, 0.0), na::zero());
let pos4 = Iso3::new(Vec3::new(0.5, 0.5, 0.0), na::zero());

let shape1 = Ball::new(0.5);
let shape2 = Ball::new(0.5);
let shape3 = Ball::new(0.5);
let shape4 = Ball::new(0.5);

let obj1 = Rc::new(RefCell::new(WithAABB(pos1, shape1)));
let obj2 = Rc::new(RefCell::new(WithAABB(pos2, shape2)));
let obj3 = Rc::new(RefCell::new(WithAABB(pos3, shape3)));
let obj4 = Rc::new(RefCell::new(WithAABB(pos4, shape4)));

/*
 * Create the broad phase.
 */
let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher::new();
let mut bf = DBVTBroadPhase::new(dispatcher, 0.2);

bf.add(obj1.clone());
bf.add(obj2.clone());
bf.add(obj3.clone());
bf.add(obj4.clone());

bf.update();

assert!(bf.num_interferences() == 6);

// Deactivate everybody.
bf.deactivate(&obj1);
bf.deactivate(&obj2);
bf.deactivate(&obj3);
bf.deactivate(&obj4);

// Deactivated bodies do not interfere with each other.
assert_eq!(bf.num_interferences(), 0);

// Reactivate everybody.
bf.activate(&obj1, |_, _, _| { });
bf.activate(&obj2, |_, _, _| { });
bf.activate(&obj3, |_, _, _| { });
bf.activate(&obj4, |_, _, _| { });

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.remove(&obj1);
bf.remove(&obj2);

// Update the broad phase.
bf.update();

assert!(bf.num_interferences() == 1)
```
