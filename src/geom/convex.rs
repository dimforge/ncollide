//! 
//! Support mapping based Convex polytope.
//!
use nalgebra::na::Cast;

/**
 * Set of point assumed to form a convex polytope.
 * 
 *   - `V`: type of the polytope points.
 *   - `N`: type of the result of a dot product between two points.
 */
#[deriving(Clone)]
pub struct Convex<N, V> {
    priv pts:    ~[V],
    priv margin: N
}

impl<N: Cast<f32>, V> Convex<N, V> {
    /**
     * Creates a polytope from a set of point. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new(pts: ~[V]) -> Convex<N, V> {
        Convex::new_with_margin(pts, Cast::from(0.04))
    }

    /**
     * Creates a polytope from a set of point and a custom margin. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new_with_margin(pts: ~[V], margin: N) -> Convex<N, V> {
        Convex {
            pts:    pts,
            margin: margin
        }
    }
}

impl<N: Clone, V> Convex<N, V> {
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
