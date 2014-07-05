//! Definition of parametric surfaces.

#[cfg(dim3)]
#[doc(inline)]
pub use parametric::parametric::ParametricSurface;

#[doc(hidden)]
pub mod parametric;
#[cfg(dim3)]
mod parametric_ball;
