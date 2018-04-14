//! 2d line strip, 3d triangle mesh, and nd subsimplex mesh.

use std::sync::Arc;

use na::{self, Point2, Point3};
use partitioning::BVT;
use bounding_volume::AABB;
use shape::{BaseMesh, CompositeShape, Shape, Triangle};
use math::{Isometry, Point};

/// Shape commonly known as a 2d line strip or a 3d triangle mesh.
pub struct TriMesh<N: Real> {
    mesh: BaseMesh<P, Point3<usize>, Triangle<N>>,
}

impl<N: Real> Clone for TriMesh<P> {
    fn clone(&self) -> TriMesh<P> {
        TriMesh {
            mesh: self.mesh.clone(),
        }
    }
}

impl<N: Real> TriMesh<P> {
    /// Builds a new mesh.
    pub fn new(
        vertices: Arc<Vec<Point<N>>>,
        indices: Arc<Vec<Point3<usize>>>,
        uvs: Option<Arc<Vec<Point2<N>>>>,
        normals: Option<Arc<Vec<Vector<N>>>>,
    ) -> TriMesh<P> {
        TriMesh {
            mesh: BaseMesh::new(vertices, indices, uvs, normals),
        }
    }

    /// The base representation of this mesh.
    #[inline]
    pub fn base_mesh(&self) -> &BaseMesh<P, Point3<usize>, Triangle<N>> {
        &self.mesh
    }

    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &Arc<Vec<Point<N>>> {
        self.mesh.vertices()
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<N>] {
        self.mesh.bounding_volumes()
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices(&self) -> &Arc<Vec<Point3<usize>>> {
        self.mesh.indices()
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Arc<Vec<Point2<N>>>> {
        self.mesh.uvs()
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals(&self) -> &Option<Arc<Vec<Vector<N>>>> {
        self.mesh.normals()
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        self.mesh.bvt()
    }
}

impl<N: Real> TriMesh<P> {
    /// Gets the i-th mesh element.
    #[inline]
    pub fn triangle_at(&self, i: usize) -> Triangle<N> {
        self.mesh.element_at(i)
    }
}

impl<N: Real> CompositeShape<P, M> for TriMesh<P> {
    #[inline]
    fn nparts(&self) -> usize {
        self.mesh.indices().len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(usize, &Isometry<N>, &Shape<N>)) {
        let one: M = na::one();

        self.map_transformed_part_at(i, &one, f)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &Isometry<N>, f: &mut FnMut(usize, &Isometry<N>, &Shape<N>)) {
        let element = self.triangle_at(i);

        f(i, m, &element)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bounding_volumes()[i].clone()
    }

    #[inline]
    fn bvt(&self) -> &BVT<usize, AABB<N>> {
        self.bvt()
    }
}
