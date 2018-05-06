pub use self::to_polyline::ToPolyline;

#[doc(hidden)]
pub mod to_polyline;

mod ball_to_polyline;
mod capsule_to_polyline;
mod cuboid_to_polyline;
mod segment_to_polyline;
mod triangle_to_polyline;
