# Frequently Asked Questions

#### How should I write and pronounce **ncollide**?

It is written without capitals (no _Ncollide_, _nCollide_, nor _NcOlLiDe_),
without dashes (no _n-collide_), and without spaces (no _n collide_): it is
just as simple as **ncollide**.

It is pronounced starting with the letter _n_ `/ɛn/` followed by the world
_collide_ `/kəˈlaɪd/`.

--------

#### What are the units used by **ncollide**?

The units used by **ncollide** are left to your imagination. They can as well
be kilometers, miles, litters, luxes, or zablotrixes. It all depends on your
application. Just try to choose your units such that the quantities (especially
distances) remain close to 1.0. Therefore it is generally a **very bad idea**
to choose pixels are your principal distance unit. In addition, huge size
ratios (_big_ objects interacting with _small_ objects) should be avoided. This
is very prone to rounding errors.

The recommended units are those given by the [International System of
Units](http://en.wikipedia.org/wiki/International_System_of_Units) − that is −
meters, kilograms, seconds, and so on.

--------

#### The compiler claims the trait method `.foo(...)` does not exists!

Make sure you imported the trait on the module that uses one of its methods!
For example, the following cannot work:


```rust
extern crate "ncollide3df32" as ncollide;

use ncollide::geom::Ball;

fn main() {
    let ball = Ball::new(1.0);

    ball.to_trimesh();
    //   ^^^^^^^^^^
    // Type `Ball` does not implement any method in scope named `to_trimesh`.
}
```

To help the compiler find `to_trimesh`, the `procedural::ToTriMesh` trait must
be imported explicitly:

```rust
extern crate "ncollide3df32" as ncollide;

use ncollide::geom::Ball;
use ncollide::procedural::ToTriMesh; // needed to call `.to_trimesh()`.

fn main() {
    let ball = Ball::new(1.0);

    ball.to_trimesh();
}
```

If this fails, then the method might as well not exist. Double check the [API
documentation](../index.html#about_this_guide) then feel free to open an issue
if this is a missing feature!

--------

#### Do I need any kind permission to reuse the figures of this guide?

Nope! All of them were created by hand using
[Inkscape](http://www.inkscape.org/), or generated with
[nrays](http://github.com/sebcrozet/nrays) or
[kiss3d](http://github.com/sebcrozet/kiss3d). We know how time-consuming
creating those kind of illustration can be so feel free to modify/publish them
anywhere you want without asking or telling anybody.
