<p align="center">
    <a href="https://discord.gg/vt9DJSW">
        <img src="https://img.shields.io/discord/507548572338880513.svg?logo=discord&colorB=7289DA">
    </a>
    <a href="https://crates.io/crates/ncollide2d">
         <img src="http://meritbadge.herokuapp.com/ncollide2d?style=flat-square" alt="crates.io">
    </a>
    <a href="https://crates.io/crates/ncollide3d">
         <img src="http://meritbadge.herokuapp.com/ncollide3d?style=flat-square" alt="crates.io">
    </a>
    <a href="https://circleci.com/gh/dimforge/ncollide">
        <img src="https://circleci.com/gh/dimforge/ncollide.svg?style=svg" alt="Build status">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="http://ncollide.org/rustdoc/ncollide2d">2D Documentation</a> | <a href="http://ncollide.org/rustdoc/ncollide3d">3D Documentation</a> | <a href="http://ncollide.org">User Guide</a> | <a href="https://discourse.nphysics.org">Forum</a>
    </strong>
</p>


⚠️**This crate is now passively-maintained. It is being superseded by the [Parry](https://parry.rs) project.**⚠️

ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with
the rust programming language.

The official user guide is available [here](http://ncollide.org).
The rustdoc documentation is available [for 3D](http://ncollide.org/rustdoc/ncollide3d) and [for 2D](http://ncollide.org/rustdoc/ncollide2d).

## Compilation
You will need the last stable build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add one the following (or both) to your `Cargo.toml` file:

```
[dependencies]
ncollide2d = "0.23" # For 2D collision detection.
ncollide3d = "0.23" # For 3D collision detection.
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
