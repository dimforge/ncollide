//! Definition of parametric surfaces.

// #[cfg(feature = "3d")]
#[doc(inline)]
pub use parametric::parametric::ParametricSurface;

#[doc(hidden)]
pub mod parametric;
#[cfg(feature = "3d")]
mod parametric_ball;
#[cfg(feature = "3d")]
mod parametric_torus;
