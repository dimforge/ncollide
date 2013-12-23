//!
//! 2d line strip, 3d triangle Mesh, and nd subsimplex mesh.
//!

use std::num::{One, Zero};
use extra::arc::Arc;
use nalgebra::na::{VecExt, AlgebraicVecExt, Rotate, Transform, Cast,
                   Translation, AbsoluteRotate, Dim};
use nalgebra::na;
use ray::Ray;
use partitioning::bvt;
use partitioning::bvt::BVT;
use bounding_volume::{HasAABB, AABB, LooseBoundingVolume};
use partitioning::bvt_visitor::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use implicit::HasMargin;
use geom::{Geom, ConcaveGeom};

pub trait MeshElement<N, V> {
    fn nvertices(unused: Option<Self>) -> uint;
    fn new_with_vertices_and_indices(&[V], &[uint], N) -> Self;
}

// XXX: sadly, M and II seem to be needed to avoid an UnconstrainedType when trying to use it…
/// Geometry commonly known as a 2d line strip or a 3d triangle mesh.
pub struct Mesh<N, V, M, II, E> {
    priv bvt:      BVT<uint, AABB<N, V>>,
    priv bvs:      ~[AABB<N, V>],
    priv margin:   N,
    priv vertices: Arc<~[V]>,
    priv indices:  Arc<~[uint]>,
    priv uvs:      Option<Arc<~[(N, N)]>>
}

impl<N: Clone + Send + Freeze, V: Clone + Send + Freeze, M, II, E> Clone for Mesh<N, V, M, II, E> {
    fn clone(&self) -> Mesh<N, V, M, II, E> {
        Mesh {
            bvt:      self.bvt.clone(),
            bvs:      self.bvs.clone(),
            margin:   self.margin.clone(),
            vertices: self.vertices.clone(),
            indices:  self.indices.clone(),
            uvs:      self.uvs.clone()
        }
    }
}

impl<N: Send + Freeze + Clone + Cast<f32> + Algebraic + Signed + Ord + Bounded + Primitive,
     V: Send + Freeze + Clone + AlgebraicVecExt<N>,
     M: One,
     II,
     E: Geom<N, V, M, II> + MeshElement<N, V>>
Mesh<N, V, M, II, E> {
    /// Builds a new mesh with a default margin of 0.04.
    pub fn new(vertices: Arc<~[V]>,
               indices:  Arc<~[uint]>,
               uvs:      Option<Arc<~[(N, N)]>>)
               -> Mesh<N, V, M, II, E> {
        Mesh::new_with_margin(vertices, indices, uvs, na::cast(0.04))
    }

    /// Builds a new mesh with a custom margin.
    pub fn new_with_margin(vertices: Arc<~[V]>,
                           indices:  Arc<~[uint]>,
                           uvs:      Option<Arc<~[(N, N)]>>,
                           margin:   N)
                           -> Mesh<N, V, M, II, E> {
        assert!(indices.get().len() % MeshElement::nvertices(None::<E>) == 0);
        uvs.as_ref().map(|uvs| assert!(uvs.get().len() == vertices.get().len()));

        let mut leaves = ~[];
        let mut bvs    = ~[];

        {
            let vs = vertices.get();
            let is = indices.get();

            for (i, is) in is.chunks(MeshElement::nvertices(None::<E>)).enumerate() {
                let vs: &[V] = *vs;
                let element: E = MeshElement::new_with_vertices_and_indices(vs, is, margin.clone());
                // loosen for better persistancy
                let id = na::one();
                let bv= element.aabb(&id).loosened(na::cast(0.04));
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_with_partitioner(leaves, bvt::kdtree_partitioner);

        Mesh {
            bvt:      bvt,
            bvs:      bvs,
            margin:   margin,
            vertices: vertices,
            indices:  indices,
            uvs:      uvs
        }
    }
}

impl<N: Clone, V: Send + Freeze + Dim, M, II, E> Mesh<N, V, M, II, E> {
    /// The vertices of this subsimplex mesh.
    #[inline]
    pub fn vertices<'a>(&'a self) -> &'a Arc<~[V]> {
        &'a self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes<'a>(&'a self) -> &'a [AABB<N, V>] {
        let res: &'a [AABB<N, V>] = self.bvs;

        res
    }

    /// The indices of this subsimplex mesh.
    #[inline]
    pub fn indices<'a>(&'a self) -> &'a Arc<~[uint]> {
        &'a self.indices
    }

    /// The texture coordinates of this subsimplex mesh.
    #[inline]
    pub fn uvs<'a>(&'a self) -> &'a Option<Arc<~[(N, N)]>> {
        &'a self.uvs
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt<'a>(&'a self) -> &'a BVT<uint, AABB<N, V>> {
        &'a self.bvt
    }

    /// The collision margin used by this mesh.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     V:  Send + Freeze + Clone + Zero + AlgebraicVecExt<N>,
     M:  Clone + Mul<M, M> + Translation<V> + AbsoluteRotate<V> + Transform<V> + Rotate<V> + One,
     II: Zero + Add<II, II>,
     E:  Geom<N, V, M, II> + MeshElement<N, V>>
ConcaveGeom<N, V, M, II> for Mesh<N, V, M, II, E> {
    #[inline(always)]
    fn map_part_at(&self, i: uint, f: |&M, &Geom<N, V, M, II>| -> ()) {
        let one: M = na::one();

        self.map_transformed_part_at(&one, i, f);
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, m: &M, i: uint, f: |&M, &Geom<N, V, M, II>| -> ()) {
        let vs: &[V] = *self.vertices.get();
        let i        = i * MeshElement::nvertices(None::<E>);
        let is       = self.indices.get().slice(i, i + MeshElement::nvertices(None::<E>));

        let element: E = MeshElement::new_with_vertices_and_indices(vs, is, self.margin());

        f(m, &element as &Geom<N, V, M, II>);
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB<N, V>, out: &mut ~[uint]) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray<V>, out: &mut ~[uint]) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB<N, V> {
        &'a self.bvs[i]
    }
}

// FIXME: move that to aabb_mesh.rs
impl<N: Cast<f32> + Primitive + Orderable,
     V: VecExt<N> + Clone,
     M: Translation<V> + AbsoluteRotate<V> + Transform<V>,
     II,
     E>
HasAABB<N, V, M> for Mesh<N, V, M, II, E> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        let bv              = self.bvt.root_bounding_volume().unwrap();
        let ls_center       = bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / Cast::from(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
