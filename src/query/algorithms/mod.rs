//! Algorithms needed for distance and penetration depth computation.

// pub use self::simplex::Simplex;
// pub use self::voronoi_simplex2::VoronoiSimplex2;
// pub use self::voronoi_simplex3::VoronoiSimplex3;
// pub use self::johnson_simplex::{JohnsonSimplex, JohnsonSimplexTemplate};
// pub use self::epa2::EPA2;
// pub use self::epa3::EPA3;

mod simplex;
// mod johnson_simplex;
#[cfg(feature = "dim2")]
mod voronoi_simplex2;
#[cfg(feature = "dim3")]
mod voronoi_simplex3;
pub mod gjk;
// pub mod epa3;
// pub mod epa2;
// pub mod minkowski_sampling;
