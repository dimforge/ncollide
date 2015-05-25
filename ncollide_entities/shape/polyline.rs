//! 2d line strip, 3d segment mesh, and nd subsimplex mesh.

use std::mem;
use std::sync::Arc;
use na::{Translate, Pnt2};
use partitioning::BVT;
use bounding_volume::AABB;
use shape::{Segment, BaseMesh};
use math::{Scalar, Point, Vect};

/// Shape commonly known as a 2d line strip or a 3d segment mesh.
pub struct Polyline<P: Point> {
    mesh: BaseMesh<P, Pnt2<usize>, Segment<P>>
}

impl<P: Point> Clone for Polyline<P> {
    fn clone(&self) -> Polyline<P> {
        Polyline {
            mesh: self.mesh.clone()
        }
    }
}

impl<P> Polyline<P>
    where P: Point,
          P::Vect: Translate<P> {
    /// Builds a new mesh.
    pub fn new(vertices: Arc<Vec<P>>,
               indices:  Arc<Vec<Pnt2<usize>>>,
               uvs:      Option<Arc<Vec<Pnt2<<P::Vect as Vect>::Scalar>>>>,
               normals:  Option<Arc<Vec<P::Vect>>>) // a loosening margin for the BVT.
               -> Polyline<P> {
        Polyline {
            mesh: BaseMesh::new(vertices, indices, uvs, normals)
        }
    }
}

impl<P> Polyline<P>
    where P: Point {
    /// The base representation of this mesh.
    #[inline]
    pub fn base_mesh(&self) -> &BaseMesh<P, Pnt2<usize>, Segment<P>> {
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
    pub fn indices(&self) -> &Arc<Vec<Pnt2<usize>>> {
        unsafe { mem::transmute(self.mesh.indices()) }
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Arc<Vec<Pnt2<<P::Vect as Vect>::Scalar>>>> {
        self.mesh.uvs()
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals(&self) -> &Option<Arc<Vec<P::Vect>>> {
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
