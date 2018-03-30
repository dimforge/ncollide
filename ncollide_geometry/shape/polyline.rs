//! 2d line strip, 3d segment mesh, and nd subsimplex mesh.

use std::mem;
use std::sync::Arc;

use na::{self, Point2};
use partitioning::BVT;
use bounding_volume::AABB;
use shape::{BaseMesh, CompositeShape, Segment, Shape};
use math::{Isometry, Point};

/// Shape commonly known as a 2d line strip or a 3d segment mesh.
pub struct Polyline<P: Point> {
    mesh: BaseMesh<P, Point2<usize>, Segment<P>>,
}

impl<P: Point> Clone for Polyline<P> {
    fn clone(&self) -> Polyline<P> {
        Polyline {
            mesh: self.mesh.clone(),
        }
    }
}

impl<P: Point> Polyline<P> {
    /// Builds a new mesh.
    pub fn new(
        vertices: Arc<Vec<P>>,
        indices: Arc<Vec<Point2<usize>>>,
        uvs: Option<Arc<Vec<Point2<P::Real>>>>,
        normals: Option<Arc<Vec<P::Vector>>>,
    ) -> Polyline<P> {
        Polyline {
            mesh: BaseMesh::new(vertices, indices, uvs, normals),
        }
    }
}

impl<P: Point> Polyline<P> {
    /// The base representation of this mesh.
    #[inline]
    pub fn base_mesh(&self) -> &BaseMesh<P, Point2<usize>, Segment<P>> {
        &self.mesh
    }

    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &Arc<Vec<P>> {
        self.mesh.vertices()
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        self.mesh.bounding_volumes()
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices(&self) -> &Arc<Vec<Point2<usize>>> {
        unsafe { mem::transmute(self.mesh.indices()) }
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Arc<Vec<Point2<P::Real>>>> {
        self.mesh.uvs()
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals(&self) -> &Option<Arc<Vec<P::Vector>>> {
        self.mesh.normals()
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<P>> {
        self.mesh.bvt()
    }
}

impl<P: Point> Polyline<P> {
    /// Gets the i-th mesh element.
    #[inline]
    pub fn segment_at(&self, i: usize) -> Segment<P> {
        self.mesh.element_at(i)
    }
}

impl<P: Point, M: Isometry<P>> CompositeShape<P, M> for Polyline<P> {
    #[inline]
    fn nparts(&self) -> usize {
        self.mesh.indices().len()    
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(&M, &Shape<P, M>)) {
        let one: M = na::one();

        self.map_transformed_part_at(i, &one, f)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, i: usize, m: &M, f: &mut FnMut(&M, &Shape<P, M>)) {
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
