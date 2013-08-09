//!
//! Support mapping based Plane geometry.
//!

use nalgebra::traits::scalar_op::ScalarDiv;
use bounding_volume::aabb::{HasAABB, AABB};

/**
 * Implicit description of a plane.
 *
 *   - `V`: type of the plane normal.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct Plane<V> {
    /// The plane normal.
    normal: V
}

impl<V> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: V) -> Plane<V> {
        Plane {
            normal: normal
        }
    }
}

impl<V: Clone> Plane<V> {
    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> V {
        self.normal.clone()
    }
}

impl<V: Bounded + Neg<V> + ScalarDiv<N> + Ord, N, M>
HasAABB<N, V, M> for Plane<V> {
    fn aabb(&self, _: &M) -> AABB<N, V> {
        AABB::new(-Bounded::max_value::<V>(), Bounded::max_value())
    }
}
