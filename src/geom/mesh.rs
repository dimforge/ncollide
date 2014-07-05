//!
//! 2d line strip, 3d triangle Mesh, and nd subsimplex mesh.
//!

use sync::Arc;
use nalgebra::na;
use nalgebra::na::Vec2;
use ray::Ray;
use partitioning::BVT;
use bounding_volume::{HasAABB, AABB, LooseBoundingVolume};
use partitioning::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use implicit::HasMargin;
use geom::{Geom, ConcaveGeom};
use math::{Scalar, Vect, Matrix};

#[cfg(dim2)]
use geom::Segment;
#[cfg(dim3)]
use geom::Triangle;
#[cfg(dim4)] // XXX: this is wrong
use geom::Triangle;

/// Trait implemented by elements usable on the Mesh.
///
/// Note that this trait is not useful per se since the Mesh is not parameterized by
/// the element type. However, types implementing this trait is valid as a type
/// alias for `MeshPrimitive`.
pub trait MeshElement {
    /// The number of vertices of this mesh element.
    fn nvertices(unused: Option<Self>) -> uint;
    /// Creates a new mesh element from a set of vertices and indices and the margin.
    fn new_with_vertices_and_indices(&[Vect], &[uint], Scalar) -> Self;
}

#[cfg(dim2)]
/// The primitive geometry used by a `Mesh`.
pub type MeshPrimitive = Segment;

#[cfg(dim3)]
/// The primitive geometry used by a `Mesh`.
pub type MeshPrimitive = Triangle;

#[cfg(dim4)]
/// The primitive geometry used by a `Mesh`.
pub type MeshPrimitive = Triangle; // XXX: this is wrong

/// Geometry commonly known as a 2d line strip or a 3d triangle mesh.
pub struct Mesh {
    bvt:      BVT<uint, AABB>,
    bvs:      Vec<AABB>,
    margin:   Scalar,
    vertices: Arc<Vec<Vect>>,
    indices:  Arc<Vec<uint>>,
    uvs:      Option<Arc<Vec<Vec2<Scalar>>>>,
    normals:  Option<Arc<Vec<Vect>>>,
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
    pub fn new(vertices: Arc<Vec<Vect>>,
               indices:  Arc<Vec<uint>>,
               uvs:      Option<Arc<Vec<Vec2<Scalar>>>>,
               normals:  Option<Arc<Vec<Vect>>>)
               -> Mesh {
        Mesh::new_with_margin(vertices, indices, uvs, normals, na::cast(0.04f64))
    }

    /// Builds a new mesh with a custom margin.
    pub fn new_with_margin(vertices: Arc<Vec<Vect>>,
                           indices:  Arc<Vec<uint>>,
                           uvs:      Option<Arc<Vec<Vec2<Scalar>>>>,
                           normals:  Option<Arc<Vec<Vect>>>,
                           margin:   Scalar)
                           -> Mesh {
        assert!(indices.len() % MeshElement::nvertices(None::<MeshPrimitive>) == 0);

        for uvs in uvs.iter() {
            assert!(uvs.len() == vertices.len());
        }

        let mut leaves = Vec::new();
        let mut bvs    = Vec::new();

        {
            let vs = vertices.deref();
            let is = indices.deref();

            for (i, is) in is.as_slice().chunks(MeshElement::nvertices(None::<MeshPrimitive>)).enumerate() {
                let vs: &[Vect] = vs.as_slice();
                let element: MeshPrimitive = MeshElement::new_with_vertices_and_indices(vs, is, margin.clone());
                // loosen for better persistancy
                let id = na::one();
                let bv = element.aabb(&id).loosened(margin);
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
    pub fn vertices<'a>(&'a self) -> &'a Arc<Vec<Vect>> {
        &self.vertices
    }

    /// Bounding volumes of the subsimplices.
    #[inline]
    pub fn bounding_volumes<'a>(&'a self) -> &'a [AABB] {
        self.bvs.as_slice()
    }

    /// The indices of this mesh.
    #[inline]
    pub fn indices<'a>(&'a self) -> &'a Arc<Vec<uint>> {
        &self.indices
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs<'a>(&'a self) -> &'a Option<Arc<Vec<Vec2<Scalar>>>> {
        &self.uvs
    }

    /// The normals of this mesh.
    #[inline]
    pub fn normals<'a>(&'a self) -> &'a Option<Arc<Vec<Vect>>> {
        &self.normals
    }

    /// The acceleration structure used for efficient collision detection and ray casting.
    #[inline]
    pub fn bvt<'a>(&'a self) -> &'a BVT<uint, AABB> {
        &self.bvt
    }

    /// The collision margin used by this mesh.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}

impl Mesh {
    /// Gets the i-th mesh element.
    #[inline(always)]
    pub fn element_at(&self, i: uint) -> MeshPrimitive {
        let vs: &[Vect] = self.vertices.as_slice();
        let i        = i * MeshElement::nvertices(None::<MeshPrimitive>);
        let is       = self.indices.slice(i, i + MeshElement::nvertices(None::<MeshPrimitive>));

        MeshElement::new_with_vertices_and_indices(vs, is, self.margin.clone())
    }
}

impl ConcaveGeom for Mesh {
    #[inline(always)]
    fn map_part_at<T>(&self, i: uint, f: |&Matrix, &Geom| -> T) -> T {
        let one: Matrix = na::one();

        self.map_transformed_part_at(&one, i, f)
    }

    #[inline(always)]
    fn map_transformed_part_at<T>(&self, m: &Matrix, i: uint, f: |&Matrix, &Geom| -> T) -> T{
        let element = self.element_at(i);

        f(m, &element as &Geom)
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB, out: &mut Vec<uint>) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray, out: &mut Vec<uint>) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB {
        self.bvs.get(i)
    }
}
