//! Definition of parametric surfaces.

// #[dim3]
#[doc(inline)]
pub use parametric::parametric::ParametricSurface;

#[doc(hidden)]
pub mod parametric;
#[dim3]
mod parametric_ball;
#[dim3]
mod parametric_torus;
