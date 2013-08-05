//!
//! Support mapping based transformed geometry.
//!

use std::num::Zero;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::transformation::{Transformation, Transformable};
use nalgebra::traits::translation::Translation;
use bounding_volume::aabb::{HasAABB, AABB};
use geom::implicit::Implicit;

/// Implicit representation of a geometry transformed by an affine
/// transformation. The wrapped geometry is captured by a borrowed pointer.
///
/// # Parameters:
///   * `G` - type of the shape being transformed.
///   * `M` - type of the transformation.
#[deriving(Eq, Clone, ToStr)]
pub struct TransformedRef<'self, G, M, N> {
    priv t: M,
    priv g: &'self G
}

/// Implicit representation of a geometry transformed by an affine
/// transformation. The wrapped geometry is a copy of the original one. See `TransformedRef` to
/// avoid the copy.
///
/// # Parameters:
///   * `G` - type of the shape being transformed.
///   * `M` - type of the transformation.
#[deriving(Eq, Clone, ToStr)]
pub struct Transformed<G, M, N> {
    priv t: M,
    priv g: G
}

impl<G: Implicit<V>,
     M: Rotate<V> + Transform<V>,
     V: Basis + Dot<N> + VectorSpace<N> + Ord + Clone,
     N> HasAABB<N, V> for Transformed<G, M, N> {
    fn aabb(&self) -> AABB<N, V> {
        let mut resm = Zero::zero::<V>();
        let mut resM = Zero::zero::<V>();

        do Basis::canonical_basis::<V>() |basis| {
            resm = resm + basis.scalar_mul(&basis.dot(&self.support_point(&-basis)));
            resM = resM + basis.scalar_mul(&basis.dot(&self.support_point(&basis)));
        }

        let res = AABB::new(resm, resM);

        res
    }
}

// FIXME: is there a way to not have two different structs for the pointer and
// the non-pointer version?

// implementations for TransformedRef
impl<'self, G, M, N> TransformedRef<'self, G, M, N> {
    /// Creates a transformed geometry.
    #[inline]
    pub fn new(transform: M, geometry: &'self G) -> TransformedRef<'self, G, M, N> {
        TransformedRef { t: transform, g: geometry }
    }
}

impl<'self, G: Implicit<V>, M: Rotate<V>, V: Clone, N>
Implicit<V> for TransformedRef<'self, G, M, N> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        self.t.rotate(&self.g.support_point(&self.t.inv_rotate(dir)))
    }
}

impl<'self, G, M: Mul<M, M> + Inv + Clone, N>
Transformation<M> for TransformedRef<'self, G, M, N> {
    #[inline]
    fn transformation(&self) -> M {
        self.t.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        match self.t.inverse() {
            Some(t) => t,
            None    => fail!("This TransformedRef instance was not inversible.")
        }
    }


    #[inline]
    fn transform_by(&mut self, transform: &M) {
        self.t = transform * self.t;
    }
}

// this might do some very interesting structural optimizations
impl<'self,
     G: Transformable<M, Res>,
     M: Mul<M, M>,
     Res: Transformation<M>,
     N>
Transformable<M, Res> for TransformedRef<'self, G, M, N> {
    #[inline]
    fn transformed(&self, transform: &M) -> Res {
        self.g.transformed(&(transform * self.t))
    }
}

// implementations for Transformed
impl<G, M, N> Transformed<G, M, N> {
    /// Creates a transformed geometry from a transform.
    #[inline]
    pub fn new(transform: M, geometry: G) -> Transformed<G, M, N> {
        Transformed { t: transform, g: geometry }
    }

    /// Reference to the wrapped geometry.
    #[inline]
    pub fn geom<'r>(&'r self) -> &'r G {
        &'r self.g
    }

    /// Transformation applied to the wrapped geometry.
    #[inline]
    pub fn transform<'r>(&'r self) -> &'r M {
        &'r self.t
    }
}

impl<G: Implicit<V>,
     M: Rotate<V> + Transform<V>,
     V,
     N>
Implicit<V> for Transformed<G, M, N> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        self.t.transform_vec(&self.g.support_point(&self.t.inv_rotate(dir)))
    }
}

impl<G, M: Mul<M, M> + Inv + Clone, N>
Transformation<M> for Transformed<G, M, N> {
    #[inline]
    fn transformation(&self) -> M {
        self.t.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        match self.t.inverse() {
            Some(t) => t,
            None    => fail!("This Transformed instance was not inversible.")
        }
    }

    #[inline]
    fn transform_by(&mut self, transform: &M) {
        self.t = transform * self.t;
    }
}

impl<G, M: Transform<V>, N, V> Transform<V> for Transformed<G, M, N> {
    #[inline]
    fn transform_vec(&self, v: &V) -> V {
        self.t.transform_vec(v)
    }

    #[inline]
    fn inv_transform(&self, v: &V) -> V {
        self.t.inv_transform(v)
    }
}

// this might do some very interesting structural optimizations
impl<G: Transformable<M, Res>, M: Mul<M, M>, Res: Transformation<M>, N>
Transformable<M, Res> for Transformed<G, M, N> {
    #[inline]
    fn transformed(&self, transform: &M) -> Res {
        self.g.transformed(&(transform * self.t))
    }
}

impl<G, M: Translation<V>, N, V> Translation<V> for Transformed<G, M, N> {
    // FIXME take the sub geometry in account?
    #[inline]
    fn translation(&self) -> V {
        self.t.translation()
    }

    #[inline]
    fn inv_translation(&self) -> V {
        self.t.inv_translation()
    }

    #[inline]
    fn translate_by(&mut self, t: &V) {
        self.t.translate_by(t)
    }
}
