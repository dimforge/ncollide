/*!
ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with the rust programming
language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).

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

## What is missing
Some common features are still missing:

- heightmaps
*/

#![deny(non_camel_case_types)]
#![deny(unnecessary_parens)]
#![deny(non_uppercase_statics)]
#![deny(unnecessary_qualification)]
#![deny(missing_doc)]
#![deny(unused_result)]
#![warn(unused_imports)]
#![deny(unnecessary_typecast)]
#![feature(macro_rules)]
#![feature(managed_boxes)]
#![feature(unsafe_destructor)]
#![feature(globs)]
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate ncollide2df32;
extern crate ncollide3df32;
extern crate ncollide4df32;
extern crate ncollide2df64;
extern crate ncollide3df64;
extern crate ncollide4df64;

mod f32 {
    mod d2 {
        pub use ncollide2df32::*;
    }

    mod d3 {
        pub use ncollide3df32::*;
    }

    mod d4 {
        pub use ncollide4df32::*;
    }
}

mod f64 {
    mod d2 {
        pub use ncollide2df64::*;
    }

    mod d3 {
        pub use ncollide3df64::*;
    }

    mod d4 {
        pub use ncollide4df64::*;
    }
}
