/*!
nprocgen
========

**nprocgen** is a 2 and 3-dimensional procedural generation library. Its short-term goal is to have
similar geometric features as the [Ogre Procedural Geometry
Library](http://www.ogre3d.org/tikiwiki/Ogre+Procedural+Geometry+Library).

## Compilation
You will need the last rust compiler from the master branch.
If you encounter problems, make sure you have the last version before creating an issue.

The simplest way to build **nprocgen** and all its dependencies is to do a
recursive clone:


    git clone --recursive git://github.com/sebcrozet/nprocgen.git
    cd nprocgen
    make deps
    make

## Features
**nprocgen** can generate the following geometries:

* cones, spheres, cylinders, boxes.
* rational and non-rational bezier surfaces.

## Dependencies
**nprocgen** uses [nalgebra](https://github.com/sebcrozet/nalgebra) for linear algebra. Demos are
available on the [kiss3d](https://github.com/sebcrozet/kiss3d) graphics engine project examples.
*/

#![crate_id = "nprocgen#0.1"]
#![crate_type = "lib"]
#![deny(non_camel_case_types)]
#![deny(unnecessary_parens)]
#![deny(non_uppercase_statics)]
#![deny(unnecessary_qualification)]
// XXX: reenable this: #![deny(missing_doc)]
#![deny(unused_result)]
#![deny(unnecessary_typecast)]
#![warn(visible_private_types)] // FIXME: should be denied
#![feature(globs)]
#![feature(macro_rules)]
#![feature(managed_boxes)]
#![doc(html_root_url = "http://www.rust-ci.org/nprocgen/nalgebra/doc")]

extern crate std;
extern crate collections;
extern crate nalgebra;

pub mod mesh;
pub mod triangulate;
pub mod utils;
pub mod path;

// XXX: move this somewhere else.
pub mod vec_slice;
