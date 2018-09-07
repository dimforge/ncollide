//! 2d line strip, 3d triangle mesh, and nd subsimplex mesh.

use bounding_volume::AABB;
use math::{Isometry, Point};
use na::{Point2, Point3, Real};
use partitioning::DBVT;
use shape::{CompositeShape, Shape, Triangle};

/// Shape commonly known as a 2d line strip or a 3d triangle mesh.
pub struct DeformableTriMesh<N: Real> {
    bvt: DBVT<usize, AABB<N>>,
    bvs: Vec<AABB<N>>,
    vertices: Vec<Point<N>>,
    indices: Vec<Point3<usize>>,
    uvs: Option<Vec<Point2<N>>>,
}

impl<N: Real> Clone for DeformableTriMesh<N> {
    fn clone(&self) -> DeformableTriMesh<N> {
        DeformableTriMesh {
            bvt: self.bvt.clone(),
            bvs: self.bvs.clone(),
            vertices: self.vertices.clone(),
            indices: self.indices.clone(),
            uvs: self.uvs.clone(),
        }
    }
}

impl<N: Real> DeformableTriMesh<N> {
    /// Builds a new mesh.
    pub fn new(
        vertices: Vec<Point<N>>,
        indices: Vec<Point3<usize>>,
        uvs: Option<Vec<Point2<N>>>,
    ) -> DeformableTriMesh<N> {
        let mut leaves = Vec::new();
        let mut bvs = Vec::new();

        {
            let is = &*indices;

            for (i, is) in is.iter().enumerate() {
                let triangle = Triangle::new(vertices[is.x], vertices[is.y], vertices[is.z]);
                // FIXME: loosen for better persistancy?
                let bv = triangle.aabb(&Isometry::identity());
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = DBVT::new_balanced(leaves);

        DeformableTriMesh {
            bvt: bvt,
            bvs: bvs,
            vertices: vertices,
            indices: indices,
            uvs: uvs,
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

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
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

impl<N: Real> CompositeShape<N> for DeformableTriMesh<N> {
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
    fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }
}
