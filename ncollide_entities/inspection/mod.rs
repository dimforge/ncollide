//! Traits and methods to inspect and retrieve the capabilities and representations of shapes at runtime.

#[doc(inline)]
pub use self::shape::{ShapeDesc, Shape};
use na::{Point2, Isometry2, Point3, Isometry3};

#[doc(hidden)]
pub mod shape;
mod impl_support_map_desc;
mod impl_composite_shape_desc;

/// A 2d dynamic representation object.
pub type Shape2<N> = Shape<Point2<N>, Isometry2<N>>;
/// A 3d dynamic representation object.
pub type Shape3<N> = Shape<Point3<N>, Isometry3<N>>;
/// A 2d dynamic representation descriptor.
pub type ShapeDesc2<'a, N> = ShapeDesc<'a, Point2<N>, Isometry2<N>>;
/// A 3d dynamic representation descriptor.
pub type ShapeDesc3<'a, N> = ShapeDesc<'a, Point3<N>, Isometry3<N>>;
