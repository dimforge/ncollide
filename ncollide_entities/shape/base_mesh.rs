//! A mesh generic wrt. the contained mesh element characterized by vertices.

use std::sync::Arc;
use na::{Translate, Identity, Pnt2};
use partitioning::BVT;
use bounding_volume::{HasAABB, AABB};
use math::{Scalar, Point, Vect};


/// Trait implemented by elements usable on the Mesh.
pub trait BaseMeshElement<I, P> {
    /// Creates a new mesh element from a set of vertices and indices.
    fn new_with_vertices_and_indices(&[P], &I) -> Self;
}

/// A mesh generic wrt. the contained mesh elements characterized by vertices.
pub struct BaseMesh<N, P, V, I, E> {
    bvt:      BVT<uint, AABB<P>>,
    bvs:      Vec<AABB<P>>,
    vertices: Arc<Vec<P>>,
    indices:  Arc<Vec<I>>,
    uvs:      Option<Arc<Vec<Pnt2<N>>>>,
    normals:  Option<Arc<Vec<V>>>,
}

impl<N, P, V, I, E> Clone for BaseMesh<N, P, V, I, E>
    where N: Clone,
          P: Send + Sync + Clone,
          V: Send + Sync {
    fn clone(&self) -> BaseMesh<N, P, V, I, E> {
        BaseMesh {
            bvt:      self.bvt.clone(),
            bvs:      self.bvs.clone(),
            vertices: self.vertices.clone(),
            indices:  self.indices.clone(),
            uvs:      self.uvs.clone(),
            normals:  self.normals.clone()
        }
    }
}

impl<N, P, V, I, E> BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Translate<P> + Vect<N>,
          E: BaseMeshElement<I, P> + HasAABB<P, Identity> {
    /// Builds a new mesh.
    pub fn new(vertices: Arc<Vec<P>>,
               indices:  Arc<Vec<I>>,
               uvs:      Option<Arc<Vec<Pnt2<N>>>>,
               normals:  Option<Arc<Vec<V>>>) // a loosening margin for the BVT.
               -> BaseMesh<N, P, V, I, E> {
        for uvs in uvs.iter() {
            assert!(uvs.len() == vertices.len());
        }

        let mut leaves = Vec::new();
        let mut bvs    = Vec::new();

        {
            let vs = &*vertices;
            let is = &*indices;

            for (i, is) in is.as_slice().iter().enumerate() {
                let vs: &[P] = vs.as_slice();
                let element: E = BaseMeshElement::new_with_vertices_and_indices(vs, is);
                // loosen for better persistancy
                let bv = element.aabb(&Identity::new());
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_balanced(leaves);

        BaseMesh {
            bvt:      bvt,
            bvs:      bvs,
            vertices: vertices,
            indices:  indices,
            uvs:      uvs,
            normals:  normals
        }
    }
}

impl<N, P, V, I, E> BaseMesh<N, P, V, I, E> {
    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &Arc<Vec<P>> {
        &self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        self.bvs.as_slice()
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices(&self) -> &Arc<Vec<I>> {
        &self.indices
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Arc<Vec<Pnt2<N>>>> {
        &self.uvs
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals(&self) -> &Option<Arc<Vec<V>>> {
        &self.normals
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt(&self) -> &BVT<uint, AABB<P>> {
        &self.bvt
    }
}

impl<N, P: Send + Sync, V, I, E: BaseMeshElement<I, P>> BaseMesh<N, P, V, I, E> {
    /// Gets the i-th mesh element.
    #[inline(always)]
    pub fn element_at(&self, i: uint) -> E {
        let vs: &[P] = self.vertices.as_slice();

        BaseMeshElement::new_with_vertices_and_indices(vs, &self.indices[i])
    }
}
