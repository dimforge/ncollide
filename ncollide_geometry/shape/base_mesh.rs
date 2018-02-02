//! A mesh generic wrt. the contained mesh element characterized by vertices.

use std::sync::Arc;
use std::marker::PhantomData;

use alga::general::Id;
use na::Point2;
use partitioning::BVT;
use bounding_volume::{self, HasBoundingVolume, AABB};
use math::Point;

/// Trait implemented by elements usable on the Mesh.
pub trait BaseMeshElement<I, P> {
    /// Creates a new mesh element from a set of vertices and indices.
    fn new_with_vertices_and_indices(&[P], &I) -> Self;
}

/// A mesh generic wrt. the contained mesh elements characterized by vertices.
pub struct BaseMesh<P: Point, I, E> {
    bvt: BVT<usize, AABB<P>>,
    bvs: Vec<AABB<P>>,
    vertices: Arc<Vec<P>>,
    indices: Arc<Vec<I>>,
    uvs: Option<Arc<Vec<Point2<P::Real>>>>,
    normals: Option<Arc<Vec<P::Vector>>>,
    elt: PhantomData<E>,
}

impl<P, I, E> Clone for BaseMesh<P, I, E>
where
    P: Point,
{
    fn clone(&self) -> BaseMesh<P, I, E> {
        BaseMesh {
            bvt: self.bvt.clone(),
            bvs: self.bvs.clone(),
            vertices: self.vertices.clone(),
            indices: self.indices.clone(),
            uvs: self.uvs.clone(),
            normals: self.normals.clone(),
            elt: PhantomData,
        }
    }
}

impl<P, I, E> BaseMesh<P, I, E>
where
    P: Point,
    E: BaseMeshElement<I, P> + HasBoundingVolume<Id, AABB<P>>,
{
    /// Builds a new mesh.
    pub fn new(
        vertices: Arc<Vec<P>>,
        indices: Arc<Vec<I>>,
        uvs: Option<Arc<Vec<Point2<P::Real>>>>,
        normals: Option<Arc<Vec<P::Vector>>>,
    ) -> BaseMesh<P, I, E> {
        for uvs in uvs.iter() {
            assert!(uvs.len() == vertices.len());
        }

        let mut leaves = Vec::new();
        let mut bvs = Vec::new();

        {
            let vs = &*vertices;
            let is = &*indices;

            for (i, is) in is[..].iter().enumerate() {
                let vs: &[P] = &vs[..];
                let element: E = BaseMeshElement::new_with_vertices_and_indices(vs, is);
                // loosen for better persistancy
                let bv = bounding_volume::aabb(&element, &Id::new());
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_balanced(leaves);

        BaseMesh {
            bvt: bvt,
            bvs: bvs,
            vertices: vertices,
            indices: indices,
            uvs: uvs,
            normals: normals,
            elt: PhantomData,
        }
    }
}

impl<P, I, E> BaseMesh<P, I, E>
where
    P: Point,
{
    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &Arc<Vec<P>> {
        &self.vertices
    }

    /// The number of primitives on thes mesh.
    #[inline]
    pub fn len(&self) -> usize {
        self.bvs.len()
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        &self.bvs[..]
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices(&self) -> &Arc<Vec<I>> {
        &self.indices
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Arc<Vec<Point2<P::Real>>>> {
        &self.uvs
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals(&self) -> &Option<Arc<Vec<P::Vector>>> {
        &self.normals
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<P>> {
        &self.bvt
    }
}

impl<P, I, E> BaseMesh<P, I, E>
where
    P: Point,
    E: BaseMeshElement<I, P>,
{
    /// Gets the i-th mesh element.
    #[inline(always)]
    pub fn element_at(&self, i: usize) -> E {
        let vs: &[P] = &self.vertices[..];

        BaseMeshElement::new_with_vertices_and_indices(vs, &self.indices[i])
    }
}
