//!
//! Shape composed from the union of primitives.
//!

use bounding_volume::{BoundingVolume, AABB};
use math::Isometry;
use na::{self, Real};
use partitioning::{BVHImpl, BVT};
use shape::{CompositeShape, Shape, ShapeHandle};

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
pub struct Compound<N: Real> {
    shapes: Vec<(Isometry<N>, ShapeHandle<N>)>,
    bvt: BVT<usize, AABB<N>>,
    bvs: Vec<AABB<N>>,
    start_idx: Vec<usize>,
}

impl<N: Real> Clone for Compound<N> {
    fn clone(&self) -> Compound<N> {
        Compound {
            shapes: self.shapes.clone(),
            bvt: self.bvt.clone(),
            bvs: self.bvs.clone(),
            start_idx: self.start_idx.clone(),
        }
    }
}

impl<N: Real> Compound<N> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(Isometry<N>, ShapeHandle<N>)>) -> Compound<N> {
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

impl<N: Real> Compound<N> {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(Isometry<N>, ShapeHandle<N>)] {
        &self.shapes[..]
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<N>] {
        &self.bvs[..]
    }

    /// The AABB of the i-th shape compositing this compound.
    #[inline]
    pub fn aabb_at(&self, i: usize) -> &AABB<N> {
        &self.bvs[i]
    }

    pub(crate) fn start_idx(&self) -> &[usize] {
        &self.start_idx[..]
    }
}

impl<N: Real> CompositeShape<N> for Compound<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.shapes.len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(usize, &Isometry<N>, &Shape<N>)) {
        let id = self.start_idx[i];
        let &(ref m, ref g) = &self.shapes()[i];

        f(id, m, g.as_ref())
    }

    #[inline(always)]
    fn map_transformed_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut FnMut(usize, &Isometry<N>, &Shape<N>),
    ) {
        let id = self.start_idx[i];
        let elt = &self.shapes()[i];

        f(id, &(m.clone() * elt.0.clone()), elt.1.as_ref())
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bounding_volumes()[i].clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        BVHImpl::BVT(&self.bvt)
    }
}
