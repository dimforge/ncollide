//!
//! Support mapping based reflected geometry.
//!

use geom::implicit::Implicit;

/**
 * Implicit represention of the reflection of a geometric object.
 * A reflection is obtained with the central symetry wrt the origin.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct Reflection<'self, G> {
    priv g: &'self G
}

impl<'self, G> Reflection<'self, G> {
    /// Build the reflection of a geometry. Since the representation is implicit,
    /// the reflection computation is done in constant time.
    #[inline]
    pub fn new(g: &'self G) -> Reflection<'self, G> {
        Reflection { g: g }
    }
}

impl<'self, V: Neg<V>, G: Implicit<V>> Implicit<V> for Reflection<'self, G> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        -self.g.support_point(&-dir)
    }
}

// Implement transform efficiently.
