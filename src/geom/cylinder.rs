//!
//! Support mapping based Cylinder geometry.
//!

use std::num::Zero;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use nalgebra::traits::transformation::Transform;
use bounding_volume::aabb::{HasAABB, AABB};
use bounding_volume::aabb;
use geom::implicit::Implicit;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Cylinder<N> {
    priv half_height: N,
    priv radius:      N
}

impl<N: Signed> Cylinder<N> {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cylinder along the `x` axis.
    ///     * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius
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

impl<N: Clone + Signed,
     V: Clone + Zero + Norm<N> + ScalarMul<N> + Indexable<uint, N>,
     M: Transform<V> + Rotate<V>>
Implicit<V, M> for Cylinder<N> {
    fn support_point(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(0).is_negative();

        vres.set(0, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero()
        }
        else {
            vres.scalar_mul_inplace(&self.radius);
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

impl<N: Signed + Clone,
     V: Dim + Indexable<uint, N> + Zero + Dot<N> + ScalarMul<N> + ScalarDiv<N> +
        Basis + Neg<V> + Add<V, V> + Norm<N> + Ord + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cylinder<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        aabb::implicit_shape_aabb(m, self)
    }
}
