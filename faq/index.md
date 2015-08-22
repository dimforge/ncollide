# Frequently Asked Questions

#### How should I write and pronounce **ncollide**?

It is written without capitals (not _Ncollide_, _nCollide_, or _NcOlLiDe_),
without dashes (not _n-collide_), and without spaces (not _n collide_): it is
just as simple as **ncollide**.

It is pronounced starting with the letter _n_ `/ɛn/` followed by the world
_collide_ `/kəˈlaɪd/`.

--------

#### Which units are used by **ncollide**?

This is left to your imagination. They can as well be kilometers, miles,
litters, lumens, or nyan cats. It all depends on your application. Just try to
choose your units such that the quantities (especially sizes) remain close
to 1.0. Therefore it is generally a **very bad idea** to choose one pixel as
your principal distance unit! In addition, significant size ratios (_big_
objects interacting with _small_ objects) should be avoided as they are very
prone to generate rounding errors.

The recommended units are those given by the [International System of
Units](http://en.wikipedia.org/wiki/International_System_of_Units)−that
is−meters, kilograms, seconds, and so on. But again, it depends on your
application!

--------

#### The compiler claims the trait method `.foo(...)` does not exists!

Make sure to `use` the trait if you use at least one of its methods! For
example, the following cannot work:


```rust
extern crate ncollide;

use ncollide::shape::Ball;

fn main() {
    let ball = Ball::new(1.0f32);

    ball.to_trimesh();
    //   ^^^^^^^^^^
    // Error: type `Ball` does not implement any method in scope named `to_trimesh`.
}
```

To help the compiler find `.to_trimesh()`, the `procedural::ToTriMesh` trait
must be imported explicitly:

```rust
extern crate ncollide;

use ncollide::shape::Ball;
use ncollide::procedural::ToTriMesh; // needed to call `.to_trimesh()`.

fn main() {
    let ball = Ball::new(1.0f32);

    ball.to_trimesh();
}
```

If doing so does not work, double check the [API
documentation](../doc/ncollide): you might just have mispelled
the method name.

--------

#### Do I need some kind of permission to reuse the figures of this guide?

Nope! They were all created using [Inkscape](http://www.inkscape.org/) or
generated with [nrays](http://github.com/sebcrozet/nrays),
[kiss3d](http://github.com/sebcrozet/kiss3d), and
[rust-sfml](http://github.com/JeremyLetang/rust-sfml). We know how
time-consuming creating those kinds of illustrations can be so feel free to
modify, publish, and redistribute them anywhere you want without asking or even
telling anybody.
