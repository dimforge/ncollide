//!
//! Support mapping based Cone geometry.
//!

use std::num::Zero;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::vector::AlgebraicVecExt;
use bounding_volume::aabb::{HasAABB, AABB};
use bounding_volume::aabb;
use geom::implicit::Implicit;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Cone<N> {
    priv half_height: N,
    priv radius: N,
    priv margin: N
}

impl<N: Signed + NumCast> Cone<N> {
    /// Creates a new cone.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cone along the `x` axis.
    ///     * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        Cone::new_with_margin(half_height, radius, NumCast::from(0.04))
    }

    /// Creates a new cone with a custom marin.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cone along the `x` axis.
    ///     * `radius` - the length of the cone along all other axis.
    ///     * `margin` - the  cone margin.
    pub fn new_with_margin(half_height: N, radius: N, margin: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius,
            margin:      margin
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

impl<N: Clone + Signed + Algebraic + Ord,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Cone<N> {
    #[inline]
    fn margin(&self) -> N {
        self.margin.clone()
    }

    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        vres.set(0, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero();

            if local_dir.at(0).is_negative() {
                vres.set(0, -self.half_height)
            }
            else {
                vres.set(0, self.half_height.clone())
            }
        }
        else {
            vres = vres * self.radius;
            vres.set(0, -self.half_height);

            if local_dir.dot(&vres) < local_dir.at(0) * self.half_height {
                vres = Zero::zero();
                vres.set(0, self.half_height.clone())
            }
        }

        m.transform(&vres)
    }
}

impl<N: Signed + Algebraic + Ord + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cone<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        aabb::implicit_shape_aabb(m, self)
    }
}
