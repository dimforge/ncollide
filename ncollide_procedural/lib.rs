//! Procedural mesh generation.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)] // FIXME: deny this.
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

extern crate rustc_serialize;
extern crate num_traits as num;
extern crate alga;
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

use na::{Point2, Point3};

pub mod utils;
pub mod path;
mod trimesh;
mod polyline;

mod sphere;
mod capsule;
mod cone;
mod cuboid;
mod cylinder;
mod quad;
mod bezier;

/// A 3D triangle mesh.
pub type TriMesh3<N> = TriMesh<Point3<N>>;
/// A 3D polyline.
pub type Polyline3<N> = Polyline<Point3<N>>;

/// A 2D triangle mesh.
pub type TriMesh2<N> = TriMesh<Point2<N>>;
/// A 2D polyline.
pub type Polyline2<N> = Polyline<Point2<N>>;
