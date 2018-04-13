//!
//! Shape composed from the union of primitives.
//!

use std::ops::Mul;

use na;

use bounding_volume::{BoundingVolume, AABB};
use partitioning::BVT;
use shape::{CompositeShape, Shape, ShapeHandle};
use math::{Isometry, Point};

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
pub struct Compound<P: Point, M: Isometry<P>> {
    shapes: Vec<(M, ShapeHandle<P, M>)>,
    bvt: BVT<usize, AABB<P>>,
    bvs: Vec<AABB<P>>,
    start_idx: Vec<usize>,
}

impl<P: Point, M: Isometry<P>> Clone for Compound<P, M> {
    fn clone(&self) -> Compound<P, M> {
        Compound {
            shapes: self.shapes.clone(),
            bvt: self.bvt.clone(),
            bvs: self.bvs.clone(),
            start_idx: self.start_idx.clone(),
        }
    }
}

impl<P: Point, M: Isometry<P>> Compound<P, M> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(M, ShapeHandle<P, M>)>) -> Compound<P, M> {
        let mut bvs = Vec::new();
        let mut leaves = Vec::new();
        let mut start_idx = Vec::new();
        let mut start_id = 0;

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            // loosen for better persistancy
            let bv = shape.as_ref().aabb(delta).loosened(na::convert(0.04f64));

            bvs.push(bv.clone());
            leaves.push((i, bv));
            start_idx.push(start_id);

            if let Some(comp) = shape.as_composite_shape() {
                start_id += comp.nparts();
            } else {
                start_id += 1;
            }
        }

        let bvt = BVT::new_balanced(leaves);

        Compound {
            shapes: shapes,
            bvt: bvt,
            bvs: bvs,
            start_idx,
        }
    }
}

impl<P: Point, M: Isometry<P>> Compound<P, M> {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(M, ShapeHandle<P, M>)] {
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

    pub(crate) fn start_idx(&self) -> &[usize] {
        &self.start_idx[..]
    }
}

impl<P: Point, M: Isometry<P>> CompositeShape<P, M> for Compound<P, M> {
    #[inline]
    fn nparts(&self) -> usize {
        self.shapes.len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(usize, &M, &Shape<P, M>)) {
        let id = self.start_idx[i];
        let &(ref m, ref g) = &self.shapes()[i];

        f(id, m, g.as_ref())
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &M, f: &mut FnMut(usize, &M, &Shape<P, M>)) {
        let id = self.start_idx[i];
        let elt = &self.shapes()[i];

        f(id, &(m.clone() * elt.0.clone()), elt.1.as_ref())
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<P> {
        self.bounding_volumes()[i].clone()
    }

    #[inline]
    fn bvt(&self) -> &BVT<usize, AABB<P>> {
        self.bvt()
    }
}
