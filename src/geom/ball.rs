//!
//! Support mapping based Ball geometry.
//!

use std::num::One;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarAdd, ScalarSub};
use nalgebra::traits::transformation::{Transformation, Transform, Transformable};
use nalgebra::traits::translation::{Translation, Translatable};
use geom::implicit::Implicit;
use bounding_volume::aabb::{HasAABB, AABB};

/**
 * Implicit description of a ball geometry.
 * 
 *  - `N`: numeric type used for the ball radius.
 *  - `V`: type of the ball center. Typically a vector.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct Ball<N, V> {
    priv center: V,
    priv radius: N
}

impl<N, V> Ball<N, V> {
    /**
     * Creates a new ball from its radius and center.
     */
    #[inline]
    pub fn new(center: V, radius: N) -> Ball<N, V> {
        Ball { center: center, radius: radius }
    }
}

impl<N: Clone, V: Clone> Ball<N, V> {
    /**
     * The ball radius.
     */
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }

    /**
     * The ball center.
     */
    #[inline]
    pub fn center(&self) -> V {
        self.center.clone()
    }
}

impl<N, V: Norm<N> + ScalarMul<N> + Add<V, V>> Implicit<V> for Ball<N, V> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        self.center + dir.normalized().scalar_mul(&self.radius)
    }
}

impl<V: Clone + Add<V, V> + Neg<V>, N, M: One + Translation<V> + Transform<V>>
Transformation<M> for Ball<N, V> {
    #[inline]
    fn transformation(&self) -> M {
        let mut res = One::one::<M>();

        res.translate_by(&self.center);

        res
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        let mut res = One::one::<M>();

        res.translate_by(&-self.center);

        res
    }

    #[inline]
    fn transform_by(&mut self, m: &M) {
        self.center = m.transform_vec(&self.center)
    }
}

impl<N, V: Clone + Add<V, V> + Neg<V>> Translation<V> for Ball<N, V> {
    #[inline]
    fn translation(&self) -> V {
        self.center.clone()
    }

    #[inline]
    fn inv_translation(&self) -> V {
        -self.center
    }

    #[inline]
    fn translate_by(&mut self, t: &V) {
        self.center = self.center + *t
    }
}

impl<N: Clone, V: Clone + Add<V, V> + Neg<V>> Translatable<V, Ball<N, V>> for Ball<N, V> {
    #[inline]
    fn translated(&self, t: &V) -> Ball<N, V> {
        Ball::new(self.center + *t, self.radius.clone())
    }
}

impl<N: Clone, V: Clone + Add<V, V> + Neg<V>, M: One + Translation<V> + Transform<V>>
Transformable<M, Ball<N, V>> for Ball<N, V> {
    #[inline]
    fn transformed(&self, transform: &M) -> Ball<N, V> {
        Ball::new(transform.transform_vec(&self.center), self.radius.clone())
    }
}

impl<N, V: ScalarAdd<N> + ScalarSub<N> + Ord + Clone> HasAABB<V> for Ball<N, V> {
    fn aabb(&self) -> AABB<V> {
        AABB::new(self.center.scalar_sub(&self.radius),
        self.center.scalar_add(&self.radius))
    }
}
