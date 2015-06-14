//! Procedural mesh generation.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)] // FIXME: deny this.
#![feature(collections)] // Quick fix to get it to compile.
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate rustc_serialize;
extern crate num;
extern crate nalgebra as na;
extern crate ncollide_math as math;
extern crate ncollide_utils;

pub use trimesh::{TriMesh, IndexBuffer};
pub use polyline::Polyline;
pub use bezier::{bezier_surface, bezier_curve, bezier_surface_at, bezier_curve_at};
pub use capsule::capsule;
pub use cone::{unit_cone, cone};
pub use cuboid::{cuboid, unit_cuboid, rectangle, unit_rectangle};
pub use cylinder::{unit_cylinder, cylinder};
pub use quad::{quad, unit_quad, quad_with_vertices};
pub use sphere::{sphere, unit_sphere, circle, unit_circle, unit_hemisphere};

use na::{Pnt2, Pnt3};

pub mod utils;
pub mod path;
mod trimesh;
mod polyline;

mod bezier;
mod capsule;
mod cone;
mod cuboid;
mod cylinder;
mod quad;
mod sphere;

/// A 3D triangle mesh.
pub type TriMesh3<N> = TriMesh<Pnt3<N>>;
/// A 3D polyline.
pub type Polyline3<N> = Polyline<Pnt3<N>>;

/// A 2D triangle mesh.
pub type TriMesh2<N> = TriMesh<Pnt2<N>>;
/// A 2D polyline.
pub type Polyline2<N> = Polyline<Pnt2<N>>;
