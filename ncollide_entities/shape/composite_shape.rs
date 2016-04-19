use std::ops::Mul;
use na;
use math::{Point, Isometry};
use partitioning::BVT;
use bounding_volume::AABB;
use shape::{Compound, TriMesh, Polyline};
use inspection::Repr;

/// Trait implemented by shapes composed of multiple simpler shapes.
///
/// A composite shape is composed of several shapes. Typically, it is a convex decomposition of
/// a concave shape.
pub trait CompositeShape<P: Point, M> {
    /// The number of parts on this composite shapes.
    fn len(&self) -> usize;
    /// Applies a function to each sub-shape of this concave shape.
    fn map_part_at(&self, usize, &mut FnMut(&M, &Repr<P, M>));
    /// Applies a transformation matrix and a function to each sub-shape of this concave
    /// shape.
    fn map_transformed_part_at(&self, usize, m: &M, &mut FnMut(&M, &Repr<P, M>));

    // FIXME: the following two methods really are not generic enough.
    /// Gets the AABB of the shape identified by the index `i`.
    fn aabb_at(&self, i: usize) -> AABB<P>;
    /// Gets the acceleration structure of the concave shape.
    fn bvt(&self) -> &BVT<usize, AABB<P>>;
}

impl<P, M> CompositeShape<P, M> for Compound<P, M>
    where P: Point,
          M: Copy + Mul<M, Output = M> {
    #[inline]
    fn len(&self) -> usize {
        self.shapes().len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(&M, &Repr<P, M>)) {
        let &(ref m, ref g) = &self.shapes()[i];

        f(m, g.as_ref())
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &M, f: &mut FnMut(&M, &Repr<P, M>)) {
        let elt = &self.shapes()[i];

        f(&(*m * elt.0), elt.1.as_ref())
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

impl<P, M> CompositeShape<P, M> for TriMesh<P>
    where P: Point,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn len(&self) -> usize {
        self.base_mesh().len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(&M, &Repr<P, M>)) {
        let one: M = na::one();

        self.map_transformed_part_at(i, &one, f)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &M, f: &mut FnMut(&M, &Repr<P, M>)) {
        let element = self.triangle_at(i);

        f(m, &element)
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

impl<P, M> CompositeShape<P, M> for Polyline<P>
    where P: Point,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn len(&self) -> usize {
        self.base_mesh().len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(&M, &Repr<P, M>)) {
        let one: M = na::one();

        self.map_transformed_part_at(i, &one, f)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &M, f: &mut FnMut(&M, &Repr<P, M>)) {
        let element = self.segment_at(i);

        f(m, &element)
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
