# The **ncollide** testbeds
The crates `ncollide_testbed2d` and `ncollide_testbed3d` provide tools for
debugging **ncollide**. They define a `Testbed` structure that is capable of
displaying a `CollisionWorld` using [rust-sfml](https://crates.io/crates/sfml)
for 2D or [kiss3d](https://crates.io/crates/kiss3d) for 3D. If you encounter a
bug on your application, you may attempt to input your collision world to a
testbed to confirm the problem is on **ncollide** instead of on your own
engine and application logic.


Testbeds should be seen as debugging and demoing tools only. Do not expect them
to:

* Be usable to, e.g., create video games or animations with photorealistic
  graphics.
* Be able to render efficiently extremely complex scenes with millions of
  polygons.
  
On the other hand, expect them to:

* Be very simple to use (4 lines of code are enough to display any scene).
* Be interactive to allow you to experiment with different configurations.
* Display the scene actually used by **ncollide** for geometric queries.

Currently, the testbeds interactivity is quite limited. Future versions will
include a user interface to allow advanced editions of the various collision
objects.

---------

We start by describing the simplest way of creating and running a 2D testbed
application (a 3D version would be similar). You first have to depend on it
explicitly because the testbed crates are **not** subsets of the main
**ncollide** library. Thus, the `ncollide_testbed2d` dependency has to be added
to your `Cargo.toml` file and import it on your project.

```rust
extern crate ncollide_testbed2d;
```

The second step is to create your collision world as usual:

```rust
let cuboid = ShapeHandle2::new(Cuboid::new(Vector2::new(1.0f32, 1.0));

let mut world = CollisionWorld::new(0.02, true);
world.add(0, na::one(), cuboid, CollisionGroups::new(), QueryType::Contact(0.0), ());
```

Then, instantiate the testbed with your collision world as input:

```rust
let mut testbed = Testbed::new(world);
```

Finally, we can update the testbed on a loop using the `.step()` method. This
will automatically handle inputs, update the collision world, and render the
scene. If the render window is closed, the `.step()` methods will return
`None`. Inside of the loop, you are free to apply any modification to the
collision world: they will be automatically detected by the testbed at the next
update. Note that you do not have to call `world.update()` your self. This is
automatically done by the testbed.

```rust
// While the testbed window is open...
while let Some(world) = testbed.step() {
    // ... modify things.
}
```

That's all! The testbed will automatically inspect the content of your
collision world and create a graphical representation of each collision object.
Each object is given a random color. This may be changed using the
`.set_color(uid, color)` method where `uid` designs the identifier you chose
for the collision object.
