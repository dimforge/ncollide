//! 
//! Support mapping based Convex polytope.
//!
use std::vec_ng::Vec;
use nalgebra::na::Cast;
use math::{Scalar, Vector};

/// Set of point assumed to form a convex polytope.
#[deriving(Clone)]
pub struct Convex {
    priv pts:    Vec<Vector>,
    priv margin: Scalar
}

impl Convex {
    /// Creates a polytope from a set of point. Those points are assumed to form
    /// a convex polytope: convexity is not checked.
    #[inline]
    pub fn new(pts: Vec<Vector>) -> Convex {
        Convex::new_with_margin(pts, Cast::from(0.04))
    }

    /// Creates a polytope from a set of point and a custom margin. Those points are assumed to
    /// form a convex polytope: convexity is not checked.
    #[inline]
    pub fn new_with_margin(pts: Vec<Vector>, margin: Scalar) -> Convex {
        Convex {
            pts:    pts,
            margin: margin
        }
    }
}

impl Convex {
    /// The list of points of this convex polytope.
    #[inline]
    pub fn pts<'a>(&'a self) -> &'a [Vector] {
        self.pts.as_slice()
    }

    /// The margin surrounding this convex polytope.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
