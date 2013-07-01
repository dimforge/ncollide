ncollide
========

**ncollide** is a n-dimensional collision detection library written with the
rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).


## Compilation
You will need the last rust compiler from the master branch.
I pull the compiler and fix my code almost every days. If you encounter
problems, make sure you have the last version.

The simplest way to build **ncollide** and all its dependencies is to do a
recursive clone:


    git clone --recursive git://github.com/sebcrozet/ncollide.git
    cd ncollide
    make deps
    make
    
## Features
Almost nothing:

- ball vs. ball collision detection,
- plane vs. any convex object collision detection.

And various traits for collision detectors and broad phase collision detection.

## What is missing
Almost everything:

- collision detection between arbitrary convex object (nearly done)
- efficient broad phases (nearly done)
- composite geometries
- heightmaps and simplex meshes (eg. triangle mesh for 3d and line strips for
  2d)
- ray-casting
- continuous collision detection
