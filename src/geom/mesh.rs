//! 2d line strip, 3d triangle Mesh, and nd subsimplex mesh.

use std::num::One;
use std::sync::Arc;
use na::{Translate, Rotate, Transform, AbsoluteRotate, Translation, Identity, Pnt2};
use na;
use ray::Ray;
use partitioning::BVT;
use bounding_volume::{HasAABB, HasBoundingSphere, AABB};
use partitioning::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use geom::{Geom, ConcaveGeom};
use ray::RayCast;
use math::{Scalar, Point, Vect};


/// Trait implemented by elements usable on the Mesh.
pub trait MeshElement<P> {
    /// The number of vertices of this mesh element.
    fn nvertices(unused: Option<Self>) -> uint;
    /// Creates a new mesh element from a set of vertices and indice.
    fn new_with_vertices_and_indices(&[P], &[uint]) -> Self;
}

/// Geometry commonly known as a 2d line strip or a 3d triangle mesh.
pub struct Mesh<N, P, V, E: MeshElement<P>> {
    bvt:      BVT<uint, AABB<P>>,
    bvs:      Vec<AABB<P>>,
    vertices: Arc<Vec<P>>,
    indices:  Arc<Vec<uint>>,
    uvs:      Option<Arc<Vec<Pnt2<N>>>>,
    normals:  Option<Arc<Vec<V>>>,
}

impl<N, P, V, E> Clone for Mesh<N, P, V, E>
    where N: Scalar,
          P: Send + Sync + Clone,
          V: Send + Sync,
          E: MeshElement<P> {
    fn clone(&self) -> Mesh<N, P, V, E> {
        Mesh {
            bvt:      self.bvt.clone(),
            bvs:      self.bvs.clone(),
            vertices: self.vertices.clone(),
            indices:  self.indices.clone(),
            uvs:      self.uvs.clone(),
            normals:  self.normals.clone()
        }
    }
}

impl<N, P, V, E> Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Translate<P> + Vect<N>,
          E: MeshElement<P> + HasAABB<P, Identity> {
    /// Builds a new mesh.
    pub fn new(vertices: Arc<Vec<P>>,
               indices:  Arc<Vec<uint>>,
               uvs:      Option<Arc<Vec<Pnt2<N>>>>,
               normals:  Option<Arc<Vec<V>>>) // a loosening margin for the BVT.
               -> Mesh<N, P, V, E> {
        assert!(indices.len() % MeshElement::nvertices(None::<E>) == 0);

        for uvs in uvs.iter() {
            assert!(uvs.len() == vertices.len());
        }

        let mut leaves = Vec::new();
        let mut bvs    = Vec::new();

        {
            let vs = vertices.deref();
            let is = indices.deref();

            for (i, is) in is.as_slice().chunks(MeshElement::nvertices(None::<E>)).enumerate() {
                let vs: &[P] = vs.as_slice();
                let element: E = MeshElement::new_with_vertices_and_indices(vs, is);
                // loosen for better persistancy
                let bv = element.aabb(&Identity::new());
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_balanced(leaves);

        Mesh {
            bvt:      bvt,
            bvs:      bvs,
            vertices: vertices,
            indices:  indices,
            uvs:      uvs,
            normals:  normals
        }
    }
}

/* // FIXME: implement for Mesh3d
impl<P, V, E: MeshElement<P>> Mesh<P, V, E> {
    /// Builds a new mesh from a triangle mesh.
    pub fn new_from_trimesh(trimesh: TriMesh<N, P, V>) -> Mesh<P, V, E> {
        let mut trimesh = trimesh;

        trimesh.unify_index_buffer();
        let coords  = Arc::new(trimesh.coords);
        let indices = trimesh.indices.unwrap_unified();

        let mut flat_indices = Vec::with_capacity(indices.len() * 3);

        for t in indices.iter() {
            flat_indices.push(t.x as uint);
            flat_indices.push(t.y as uint);
            flat_indices.push(t.z as uint);
        }

        let normals = trimesh.normals.map(|ns| Arc::new(ns));
        let uvs     = trimesh.uvs.map(|uvs| Arc::new(uvs));

        Mesh::new(coords, Arc::new(flat_indices), uvs, normals)
    }
}
*/

impl<N, P, V, E: MeshElement<P>> Mesh<N, P, V, E> {
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
    pub fn indices(&self) -> &Arc<Vec<uint>> {
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

impl<N, P: Send + Sync, V, E: MeshElement<P>> Mesh<N, P, V, E> {
    /// Gets the i-th mesh element.
    #[inline(always)]
    pub fn element_at(&self, i: uint) -> E {
        let vs: &[P] = self.vertices.as_slice();
        let i        = i * MeshElement::nvertices(None::<E>);
        let is       = self.indices.slice(i, i + MeshElement::nvertices(None::<E>));

        MeshElement::new_with_vertices_and_indices(vs, is)
    }
}

impl<N, P, V, M, E> ConcaveGeom<N, P, V, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: One + Rotate<V> + AbsoluteRotate<V> + Transform<P> + Translation<V>,
          E: Send + MeshElement<P> + HasAABB<P, M> + HasBoundingSphere<N, P, M> + RayCast<N, P, V, M> + Clone {
    #[inline(always)]
    fn map_part_at<T>(&self, i: uint, f: |&M, &Geom<N, P, V, M>| -> T) -> T {
        let one: M = na::one();

        self.map_transformed_part_at(&one, i, f)
    }

    #[inline(always)]
    fn map_transformed_part_at<T>(&self, m: &M, i: uint, f: |&M, &Geom<N, P, V, M>| -> T) -> T{
        let element = self.element_at(i);

        f(m, &element as &Geom<N, P, V, M>)
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB<P>, out: &mut Vec<uint>) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray<P, V>, out: &mut Vec<uint>) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at(&self, i: uint) -> &AABB<P> {
        &self.bvs[i]
    }
}
