//!
//! Shape composed from the union of primitives.
//!

use std::sync::Arc;
use na::Translate;
use na;
use bounding_volume::{HasAABB, AABB, BoundingVolume};
use partitioning::BVT;
use math::{Scalar, Point, Vect, Isometry};
use inspection::Repr;

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
pub struct Compound<N, P, V, M> {
    shapes:  Vec<(M, Arc<Box<Repr<N, P, V, M>>>)>,
    bvt:     BVT<usize, AABB<P>>,
    bvs:     Vec<AABB<P>>
}

impl<N, P, V, M> Clone for Compound<N, P, V, M>
    where N: Clone,
          P: Send + Sync + Clone,
          V: Send + Sync + Clone,
          M: Clone {
    fn clone(&self) -> Compound<N, P, V, M> {
        Compound {
            shapes: self.shapes.clone(),
            bvt:    self.bvt.clone(),
            bvs:    self.bvs.clone()
        }
    }
}

impl<N, P, V, M> Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(M, Arc<Box<Repr<N, P, V, M>>>)>) -> Compound<N, P, V, M> {
        let mut bvs    = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            // loosen for better persistancy
            let bv = shape.aabb(delta).loosened(na::cast(0.04f64));

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_balanced(leaves);

        Compound {
            shapes:  shapes,
            bvt:     bvt,
            bvs:     bvs
        }
    }
}

impl<N, P, V, M> Compound<N, P, V, M> {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(M, Arc<Box<Repr<N, P, V, M>>>)] {
        self.shapes.as_slice()
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<P>> {
        &self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        self.bvs.as_slice()
    }

    /// The AABB of the i-th shape compositing this compound.
    #[inline]
    pub fn aabb_at(&self, i: usize) -> &AABB<P> {
        &self.bvs[i]
    }
}
