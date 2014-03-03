//! 
//! Support mapping based Convex polytope.
//!
use nalgebra::na::Cast;
use math::{Scalar, Vector};

/// Set of point assumed to form a convex polytope.
#[deriving(Clone)]
pub struct Convex {
    priv pts:    ~[Vector],
    priv margin: Scalar
}

impl Convex {
    /// Creates a polytope from a set of point. Those points are assumed to form
    /// a convex polytope: convexity is not checked.
    #[inline]
    pub fn new(pts: ~[Vector]) -> Convex {
        Convex::new_with_margin(pts, Cast::from(0.04))
    }

    /// Creates a polytope from a set of point and a custom margin. Those points are assumed to
    /// form a convex polytope: convexity is not checked.
    #[inline]
    pub fn new_with_margin(pts: ~[Vector], margin: Scalar) -> Convex {
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
        let res: &'a [Vector] = self.pts;

        res
    }

    /// The margin surrounding this convex polytope.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
