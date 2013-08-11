//!
//! Support mapping based Cone geometry.
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
pub struct Cone<N> {
    priv half_height: N,
    priv radius: N
}

impl<N: Signed> Cone<N> {
    /// Creates a new cone.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cone along the `x` axis.
    ///     * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius: radius
        }
    }
}

impl<N: Clone> Cone<N> {
    /// The cone half length along the `x` axis.
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `x` axis.
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

impl<N: Clone + Signed,
     V: Clone + Zero + Norm<N> + ScalarMul<N> + Indexable<uint, N>,
     M: Transform<V> + Rotate<V>>
Implicit<V, M> for Cone<N> {
    fn support_point(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        if local_dir.at(0).is_negative() { // points toward the base 
            let mut vres = local_dir.clone();

            vres.set(0, Zero::zero());

            if vres.normalize().is_zero() {
                vres = Zero::zero()
            }
            else {
                vres.scalar_mul_inplace(&self.radius)
            }

            vres.set(0, -self.half_height);

            m.transform(&vres)
        }
        else { // points toward the pointy thing
            let mut vres = Zero::zero::<V>();

            vres.set(0, self.half_height.clone());

            m.transform(&vres)
        }
    }
}

impl<N: Signed + Clone,
     V: Dim + Indexable<uint, N> + Zero + Dot<N> + ScalarMul<N> + ScalarDiv<N> +
        Basis + Neg<V> + Add<V, V> + Norm<N> + Ord + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cone<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        aabb::implicit_shape_aabb(m, self)
    }
}
