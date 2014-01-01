//! 
//! Support mapping based Convex polytope.
//!
use nalgebra::na::Cast;
use math::{N, V};

/**
 * Set of point assumed to form a convex polytope.
 * 
 *   - `V`: type of the polytope points.
 *   - `N`: type of the result of a dot product between two points.
 */
#[deriving(Clone)]
pub struct Convex {
    priv pts:    ~[V],
    priv margin: N
}

impl Convex {
    /**
     * Creates a polytope from a set of point. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new(pts: ~[V]) -> Convex {
        Convex::new_with_margin(pts, Cast::from(0.04))
    }

    /**
     * Creates a polytope from a set of point and a custom margin. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new_with_margin(pts: ~[V], margin: N) -> Convex {
        Convex {
            pts:    pts,
            margin: margin
        }
    }
}

impl Convex {
    #[inline]
    pub fn pts<'a>(&'a self) -> &'a [V] {
        let res: &'a [V] = self.pts;

        res
    }

    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}
