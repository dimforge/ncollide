//!
//! Support mapping based Ball geometry.
//!

use nalgebra::na::{AlgebraicVec, AlgebraicVecExt, ScalarSub, ScalarAdd, Translation};
use geom::{HasMargin, Implicit};
use bounding_volume::{HasAABB, AABB};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;

/**
 * Implicit description of a ball geometry.
 * 
 *  - `N`: numeric type used for the ball radius.
 */
#[deriving(Eq, ToStr, Clone, Encodable, Decodable)]
pub struct Ball<N> {
    priv radius: N
}

impl<N> Ball<N> {
    /**
     * Creates a new ball from its radius and center.
     */
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        Ball { radius: radius }
    }
}

impl<N: Clone> Ball<N> {
    /**
     * The ball radius.
     */
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

impl<N: Clone> HasMargin<N> for Ball<N> {
    #[inline]
    fn margin(&self) -> N {
        self.radius.clone()
    }
}


impl<N: Algebraic + Clone, V: AlgebraicVec<N>, M: Translation<V>> Implicit<N, V, M> for Ball<N> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, _: &V) -> V {
        m.translation()
    }
}

impl<N,
     V: AlgebraicVecExt<N> + Ord,
     M: Translation<V>>
HasAABB<N, V, M> for Ball<N> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        AABB::new(m.translation().sub_s(&self.radius),
                  m.translation().add_s(&self.radius))
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Ball<N> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
