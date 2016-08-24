<p align="center">
    <a href="https://crates.io/crates/ncollide">
         <img src="http://meritbadge.herokuapp.com/ncollide?style=flat-square" alt="crates.io">
    </a>
    <a href="https://travis-ci.org/sebcrozet/ncollide">
        <img src="https://travis-ci.org/sebcrozet/ncollide.svg?branch=master" alt="Build status">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="http://ncollide.org/rustdoc/ncollide">Documentation</a> | <a href="http://ncollide.org">User Guide</a> | <a href="http://users.nphysics.org">Forum</a>
    </strong>
</p>

ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with
the rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries. It might work with higher
dimensions (never tried).

The official user guide is available [here](http://ncollide.org).
The rustdoc documentation is available [here](http://ncollide.org/rustdoc/ncollide).

## Compilation
You will need the last stable build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```
[dependencies]
ncollide = "0.10.*"
```


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
