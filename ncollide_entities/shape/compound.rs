//!
//! Shape composed from the union of primitives.
//!

use std::sync::Arc;
use na::Translate;
use na;
use bounding_volume::{self, AABB, BoundingVolume};
use partitioning::BVT;
use math::{Point, Vect, Isometry};
use inspection::Repr;

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
pub struct Compound<P: Point, M> {
    shapes:  Vec<(M, Arc<Box<Repr<P, M>>>)>,
    bvt:     BVT<usize, AABB<P>>,
    bvs:     Vec<AABB<P>>
}

impl<P, M> Clone for Compound<P, M>
    where P: Point,
          M: Clone {
    fn clone(&self) -> Compound<P, M> {
        Compound {
            shapes: self.shapes.clone(),
            bvt:    self.bvt.clone(),
            bvs:    self.bvs.clone()
        }
    }
}

impl<P, M> Compound<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(M, Arc<Box<Repr<P, M>>>)>) -> Compound<P, M> {
        let mut bvs    = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            // loosen for better persistancy
            let bv = bounding_volume::aabb(&***shape, delta).loosened(na::cast(0.04f64));

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

impl<P, M> Compound<P, M>
    where P: Point {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(M, Arc<Box<Repr<P, M>>>)] {
        &self.shapes[..]
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<P>> {
        &self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        &self.bvs[..]
    }

    /// The AABB of the i-th shape compositing this compound.
    #[inline]
    pub fn aabb_at(&self, i: usize) -> &AABB<P> {
        &self.bvs[i]
    }
}
