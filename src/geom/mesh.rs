//!
//! 2d line strip, 3d triangle Mesh, and nd subsimplex mesh.
//!

use extra::arc::Arc;
use nalgebra::na::{Transform, Translation, AbsoluteRotate};
use nalgebra::na;
use ray::Ray;
use partitioning::bvt::BVT;
use bounding_volume::{HasAABB, AABB, LooseBoundingVolume};
use partitioning::bvt_visitor::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use implicit::HasMargin;
use geom::{Geom, ConcaveGeom};
use math::{N, V, M};

#[cfg(dim2)]
use geom::Segment;
#[cfg(dim3)]
use geom::Triangle;
#[cfg(dim4)] // XXX: this is wrong
use geom::Triangle;

pub trait MeshElement {
    fn nvertices(unused: Option<Self>) -> uint;
    fn new_with_vertices_and_indices(&[V], &[uint], N) -> Self;
}

#[cfg(dim2)]
pub type E = Segment;

#[cfg(dim3)]
pub type E = Triangle;

#[cfg(dim4)]
pub type E = Triangle; // XXX: this is wrong

/// Geometry commonly known as a 2d line strip or a 3d triangle mesh.
pub struct Mesh {
    priv bvt:      BVT<uint, AABB>,
    priv bvs:      ~[AABB],
    priv margin:   N,
    priv vertices: Arc<~[V]>,
    priv indices:  Arc<~[uint]>,
    priv uvs:      Option<Arc<~[(N, N, N)]>>,
    priv normals:  Option<Arc<~[V]>>,
}

impl Clone for Mesh {
    fn clone(&self) -> Mesh {
        Mesh {
            bvt:      self.bvt.clone(),
            bvs:      self.bvs.clone(),
            margin:   self.margin.clone(),
            vertices: self.vertices.clone(),
            indices:  self.indices.clone(),
            uvs:      self.uvs.clone(),
            normals:  self.normals.clone()
        }
    }
}

impl Mesh {
    /// Builds a new mesh with a default margin of 0.04.
    pub fn new(vertices: Arc<~[V]>,
               indices:  Arc<~[uint]>,
               uvs:      Option<Arc<~[(N, N, N)]>>,
               normals:  Option<Arc<~[V]>>)
               -> Mesh {
        Mesh::new_with_margin(vertices, indices, uvs, normals, na::cast(0.04))
    }

    /// Builds a new mesh with a custom margin.
    pub fn new_with_margin(vertices: Arc<~[V]>,
                           indices:  Arc<~[uint]>,
                           uvs:      Option<Arc<~[(N, N, N)]>>,
                           normals:  Option<Arc<~[V]>>,
                           margin:   N)
                           -> Mesh {
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
                let bv= element.aabb(&id).loosened(margin);
                leaves.push((i, bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_kdtree(leaves);

        Mesh {
            bvt:      bvt,
            bvs:      bvs,
            margin:   margin,
            vertices: vertices,
            indices:  indices,
            uvs:      uvs,
            normals:  normals
        }
    }
}

impl Mesh {
    /// The vertices of this mesh.
    #[inline]
    pub fn vertices<'a>(&'a self) -> &'a Arc<~[V]> {
        &'a self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes<'a>(&'a self) -> &'a [AABB] {
        let res: &'a [AABB] = self.bvs;

        res
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices<'a>(&'a self) -> &'a Arc<~[uint]> {
        &'a self.indices
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs<'a>(&'a self) -> &'a Option<Arc<~[(N, N, N)]>> {
        &'a self.uvs
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals<'a>(&'a self) -> &'a Option<Arc<~[V]>> {
        &'a self.normals
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt<'a>(&'a self) -> &'a BVT<uint, AABB> {
        &'a self.bvt
    }

    /// The collision margin used by this mesh.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl Mesh {
    #[inline(always)]
    pub fn element_at(&self, i: uint) -> E {
        let vs: &[V] = *self.vertices.get();
        let i        = i * MeshElement::nvertices(None::<E>);
        let is       = self.indices.get().slice(i, i + MeshElement::nvertices(None::<E>));

        MeshElement::new_with_vertices_and_indices(vs, is, self.margin.clone())
    }
}

impl ConcaveGeom for Mesh {
    #[inline(always)]
    fn map_part_at(&self, i: uint, f: |&M, &Geom| -> ()) {
        let one: M = na::one();

        self.map_transformed_part_at(&one, i, f);
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, m: &M, i: uint, f: |&M, &Geom| -> ()) {
        let element = self.element_at(i);

        f(m, &element as &Geom);
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB, out: &mut ~[uint]) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray, out: &mut ~[uint]) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB {
        &'a self.bvs[i]
    }
}

// FIXME: move that to aabb_mesh.rs
impl HasAABB for Mesh {
    #[inline]
    fn aabb(&self, m: &M) -> AABB {
        let bv              = self.bvt.root_bounding_volume().unwrap();
        let ls_center       = bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / na::cast::<f64, N>(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
