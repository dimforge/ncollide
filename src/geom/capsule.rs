//!
//! Support mapping based Capsule geometry.
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

/// Implicit description of a capsule geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Capsule<N> {
    priv half_height: N,
    priv radius:      N
}

impl<N: Signed> Capsule<N> {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the capsule along the `x` axis.
    ///     * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius
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

impl<N: Clone + Signed,
     V: Clone + Zero + Norm<N> + ScalarMul<N> + Indexable<uint, N>,
     M: Transform<V> + Rotate<V>>
Implicit<V, M> for Capsule<N> {
    fn support_point(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(0).is_negative();

        vres.scalar_mul_inplace(&self.radius);

        let v0 = vres.at(0);

        if negative {
            vres.set(0, v0 - self.half_height)
        }
        else {
            vres.set(0, v0 + self.half_height.clone())
        }

        m.transform_vec(&vres)
    }
}

impl<N: Signed + Clone,
     V: Dim + Indexable<uint, N> + Zero + Dot<N> + ScalarMul<N> + ScalarDiv<N> +
        Basis + Neg<V> + Add<V, V> + Norm<N> + Ord + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Capsule<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        aabb::implicit_shape_aabb(m, self)
    }
}
