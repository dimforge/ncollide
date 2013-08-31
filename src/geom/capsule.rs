//!
//! Support mapping based Capsule geometry.
//!  
use std::num::Zero;
use nalgebra::vec::{Indexable, AlgebraicVecExt};
use nalgebra::mat::{Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::{Implicit, HasMargin};

/// Implicit description of a capsule geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Capsule<N> {
    priv half_height: N,
    priv radius:      N,
    priv margin:      N
}

impl<N: Signed> Capsule<N> {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the capsule along the `x` axis.
    ///     * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N, margin: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius,
            margin:      margin
        }
    }
}

impl<N: Clone> Capsule<N> {
    /// The capsule half length along the `x` axis.
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the capsule's rounded part.
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}


impl<N: Clone> HasMargin<N> for Capsule<N> {
    #[inline]
    fn margin(&self) -> N {
        self.radius.clone()
    }
}

impl<N: Clone + Signed + Algebraic,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Capsule<N> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = Zero::zero();

        if local_dir.at(0).is_negative() {
            vres.set(0, -self.half_height)
        }
        else {
            vres.set(0, self.half_height.clone())
        }

        m.transform(&vres)
    }
}

impl<N: Algebraic + Signed + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Capsule<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
