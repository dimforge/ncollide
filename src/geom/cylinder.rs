//!
//! Support mapping based Cylinder geometry.
//!

use std::num::Zero;
use nalgebra::vec::{Indexable, AlgebraicVecExt};
use nalgebra::mat::{Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::{Implicit, HasMargin};

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone, Encodable, Decodable)]
pub struct Cylinder<N> {
    priv half_height: N,
    priv radius:      N,
    priv margin:      N
}

impl<N: Signed + NumCast> Cylinder<N> {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cylinder along the `x` axis.
    ///     * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        Cylinder::new_with_margin(half_height, radius, NumCast::from(0.04))
    }

    /// Creates a new cylinder.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cylinder along the `x` axis.
    ///     * `radius` - the length of the cylinder along all other axis.
    pub fn new_with_margin(half_height: N, radius: N, margin: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius,
            margin:      margin
        }
    }
}

impl<N: Clone> Cylinder<N> {
    /// The cylinder half length along the `x` axis.
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `x` axis.
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

impl<N: Clone> HasMargin<N> for Cylinder<N> {
    #[inline]
    fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl<N: Clone + Algebraic + Signed,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Cylinder<N> {
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(0).is_negative();

        vres.set(0, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero()
        }
        else {
            vres = vres * self.radius;
        }

        if negative {
            vres.set(0, -self.half_height)
        }
        else {
            vres.set(0, self.half_height.clone())
        }

        m.transform(&vres)
    }
}

impl<N: Signed + Algebraic + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cylinder<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
