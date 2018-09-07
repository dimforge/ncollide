//! 2d line strip, 3d segment mesh, and nd subsimplex mesh.

use bounding_volume::AABB;
use math::{Isometry, Point};
use na::Real;
use partitioning::BVT;
use shape::{CompositeShape, Segment, Shape};

/// Shape commonly known as a 2d line strip or a 3d segment mesh.
#[derive(Clone)]
pub struct Polyline<N: Real> {
    bvt: BVT<usize, AABB<N>>,
    bvs: Vec<AABB<N>>,
    // FIXME: duplicate of what is already of the BVT.
    vertices: Vec<Point<N>>,
}

impl<N: Real> Polyline<N> {
    /// Builds a new mesh.
    pub fn new(vertices: Vec<Point<N>>) -> Polyline<N> {
        let mut leaves = Vec::new();
        let mut bvs = Vec::new();

        for (i, vtx) in vertices.windows(2).enumerate() {
            let element = Segment::new(vtx[0], vtx[1]);
            // FIXME: loosen for better persistancy?
            let bv = element.aabb(&Isometry::identity());
            leaves.push((i, bv.clone()));
            bvs.push(bv);
        }

        let bvt = BVT::new_balanced(leaves);

        Polyline {
            bvt: bvt,
            bvs: bvs,
            vertices: vertices,
        }
    }

    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &[Point<N>] {
        &self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<N>] {
        &self.bvs
    }

    /// Gets the i-th mesh element.
    #[inline]
    pub fn segment_at(&self, i: usize) -> Segment<N> {
        Segment::new(self.vertices[i], self.vertices[i + 1])
    }
}

impl<N: Real> CompositeShape<N> for Polyline<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.vertices.len() / 2
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(usize, &Isometry<N>, &Shape<N>)) {
        self.map_transformed_part_at(i, &Isometry::identity(), f)
    }

    #[inline(always)]
    fn map_transformed_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut FnMut(usize, &Isometry<N>, &Shape<N>),
    ) {
        let element = self.segment_at(i);

        f(i, m, &element)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bvs[i].clone()
    }

    #[inline]
    fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }
}
