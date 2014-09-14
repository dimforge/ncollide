ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with
the rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).

An on-line version of this documentation is available [here](http://ncollide.org).

## Compilation
You will need the last nightly build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file if you want the 3d version
of ncollide, using 32-bits foalting point numbers:

```
[dependencies.ncollide3df32]
git = "https://github.com/sebcrozet/ncollide"
```

Depending on the accuracy or the dimension that you need, `ncollide3df32` may
be replaced by:

* `ncollide2df32` − for 2d collision detection and 32 bits precision.
* `ncollide3df32` − for 3d collision detection and 32 bits precision.
* `ncollide4df32` − for 4d collision detection and 32 bits precision.
* `ncollide2df64` − for 2d collision detection and 64 bits precision.
* `ncollide3df64` − for 3d collision detection and 64 bits precision.
* `ncollide4df64` − for 4d collision detection and 64 bits precision.


## Features
- dynamic bounding volume tree based broad phase
- ball vs. ball collision detection,
- plane vs. any convex object collision detection.
- collision detection between arbitrary convex objects
- compound geometries
- ray-casting
- time of impact computation  for objects without rotational movement (compound vs. compound is not
  yet implemented)

And various traits for collision detectors and broad phase collision detection.
