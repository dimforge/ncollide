//! 2d line strip, 3d triangle mesh, and nd subsimplex mesh.

use bounding_volume::AABB;
use either::Either;
use math::{DIM, Isometry, Point, Vector};
use na::{Point2, Point3, Real};
use partitioning::{BVHImpl, BVT, DBVT};
use shape::{CompositeShape, DeformableShape, DeformationIndex, DeformationsType, Shape, Triangle};

/// A 3d triangle mesh.
pub struct TriMesh<N: Real> {
    bvt: Either<BVT<usize, AABB<N>>, DBVT<N, usize, AABB<N>>>,
    bvs: Vec<AABB<N>>,
    vertices: Vec<Point<N>>,
    indices: Vec<Point3<usize>>,
    uvs: Option<Vec<Point2<N>>>,
}

impl<N: Real> Clone for TriMesh<N> {
    fn clone(&self) -> TriMesh<N> {
        TriMesh {
            bvt: self.bvt.clone(),
            bvs: self.bvs.clone(),
            vertices: self.vertices.clone(),
            indices: self.indices.clone(),
            uvs: self.uvs.clone(),
        }
    }
}

impl<N: Real> TriMesh<N> {
    /// Builds a new mesh.
    pub fn new(
        vertices: Vec<Point<N>>,
        indices: Vec<Point3<usize>>,
        uvs: Option<Vec<Point2<N>>>,
        deformable: bool,
    ) -> TriMesh<N> {
        let mut leaves = Vec::new();
        let mut bvs = Vec::new();

        {
            let is = &*indices;

            for (i, is) in is.iter().enumerate() {
                let triangle = Triangle::new(vertices[is.x], vertices[is.y], vertices[is.z]);
                // FIXME: loosen for better persistency?
                let bv = triangle.aabb(&Isometry::identity());
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_balanced(leaves);

        TriMesh {
            bvt: Either::Left(bvt),
            bvs: bvs,
            vertices: vertices,
            indices: indices,
            uvs: uvs,
        }
    }

    /// The triangle mesh's AABB.
    #[inline]
    pub fn aabb(&self) -> &AABB<N> {
        match &self.bvt {
            Either::Left(bvt) => bvt.root_bounding_volume().expect("An empty TriMesh has no AABB."),
            Either::Right(dbvt) => dbvt.root_bounding_volume().expect("An empty TriMesh has no AABB."),
        }
    }

    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &Vec<Point<N>> {
        &self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<N>] {
        &self.bvs
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices(&self) -> &Vec<Point3<usize>> {
        &self.indices
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Vec<Point2<N>>> {
        &self.uvs
    }

    /// Gets the i-th mesh element.
    #[inline]
    pub fn triangle_at(&self, i: usize) -> Triangle<N> {
        let idx = self.indices[i];

        Triangle::new(
            self.vertices[idx.x],
            self.vertices[idx.y],
            self.vertices[idx.z],
        )
    }
}

impl<N: Real> CompositeShape<N> for TriMesh<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.indices.len()
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
        let element = self.triangle_at(i);

        f(i, m, &element)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bvs[i].clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        match &self.bvt {
            Either::Left(bvt) => BVHImpl::BVT(bvt),
            Either::Right(dbvt) => BVHImpl::DBVT(dbvt)
        }
    }
}

impl<N: Real> DeformableShape<N> for TriMesh<N> {
    fn deformations_type(&self) -> DeformationsType {
        DeformationsType::Vectors
    }

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N], indices: &[DeformationIndex]) {
        for id in indices {
            self.vertices[id.target].coords = Vector::from_column_slice(&coords[id.source..DIM])
        }
    }
}