//!
//! 2d line strip, 3d triangle Mesh, and nd subsimplex mesh.
//!

use extra::arc::Arc;
use nalgebra::na::{VecExt, AlgebraicVec, AlgebraicVecExt, Rotate, Transform, Cast, Identity,
                   Translation, AbsoluteRotate, Dim};
use nalgebra::na;
use partitioning::bvt;
use partitioning::bvt::BVT;
use bounding_volume::{HasAABB, AABB, LooseBoundingVolume, implicit_shape_aabb};
use geom::{HasMargin, Implicit};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;


/// A subsimplex mesh is commonly known as a 2d line strip or a 3d triangle mesh and can be used as
/// such.
pub struct SubsimplexMesh<N, V> {
    priv bvt:      BVT<uint, AABB<N, V>>,
    priv bvs:      ~[AABB<N, V>],
    priv margin:   N,
    priv vertices: Arc<~[V]>,
    priv indices:  Arc<~[uint]>,
    priv uvs:      Option<Arc<~[(N, N)]>>
}

impl<N: Send + Freeze + Clone + Cast<f32> + Algebraic + Signed + Ord + Bounded + Primitive,
     V: Send + Freeze + Clone + AlgebraicVecExt<N>>
SubsimplexMesh<N, V> {
    /// Builds a new subsimplex mesh with a default margin of 0.04.
    pub fn new(vertices: Arc<~[V]>,
               indices:  Arc<~[uint]>,
               uvs:      Option<Arc<~[(N, N)]>>)
               -> SubsimplexMesh<N, V> {
        SubsimplexMesh::new_with_margin(vertices, indices, uvs, na::cast(0.04))
    }

    /// Builds a new subsimplex mesh with a custom margin.
    pub fn new_with_margin(vertices: Arc<~[V]>,
                           indices:  Arc<~[uint]>,
                           uvs:      Option<Arc<~[(N, N)]>>,
                           margin:   N)
                           -> SubsimplexMesh<N, V> {
        assert!(indices.get().len() % na::dim::<V>() == 0);
        uvs.as_ref().map(|uvs| assert!(uvs.get().len() == vertices.get().len()));

        let mut leaves = ~[];
        let mut bvs    = ~[];

        {
            let vs = vertices.get();
            let is = indices.get();

            for (i, is) in is.chunks(na::dim::<V>()).enumerate() {
                // FIXME: the slice_to on vs is weird
                let subsimplex = Subsimplex::new(vs.slice_to(vs.len() - 1), is, margin.clone());
                let bv         = subsimplex.aabb(&Identity::new()).loosened(na::cast(0.04)); // loosen for better persistancy
                leaves.push((i * na::dim::<V>(), bv.clone()));
                bvs.push(bv);
            }
        }

        let bvt = BVT::new_with_partitioner(leaves, bvt::kdtree_partitioner);

        SubsimplexMesh {
            bvt:      bvt,
            bvs:      bvs,
            margin:   margin,
            vertices: vertices,
            indices:  indices,
            uvs:      uvs
        }
    }
}

impl<N: Clone, V: Send + Freeze + Dim> SubsimplexMesh<N, V> {
    /// The subsimplex with vertices' indices starting at the specified index.
    #[inline]
    pub fn subsimplex_at<'a>(&'a self, i: uint) -> Subsimplex<'a, N, V> {
        let vs = self.vertices.get();
        let is = self.indices.get();
        let is = is.slice(i, i + na::dim::<V>());
        // FIXME: the slice_to on vs is weird

        Subsimplex::new(vs.slice_to(vs.len() - 1), is, self.margin.clone())
    }

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

impl<N: Cast<f32> + Primitive + Orderable,
     V: VecExt<N> + Clone,
     M: Translation<V> + AbsoluteRotate<V> + Transform<V>>
HasAABB<N, V, M> for SubsimplexMesh<N, V> {
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

/// A sub simplex: a segment in 2d and a triangle in 3d.
pub struct Subsimplex<'a, N, V> {
    priv margin:   N,
    priv vertices: &'a [V],
    priv indices:  &'a [uint],
}

impl<'a, N, V: Dim> Subsimplex<'a, N, V> {
    /// Creates a new subsimplex.
    #[inline]
    pub fn new(vertices: &'a [V], indices: &'a [uint], margin: N) -> Subsimplex<'a, N, V> {
        assert!(indices.len() == na::dim::<V>());

        Subsimplex {
            margin:   margin,
            vertices: vertices,
            indices:  indices,
        }
    }
}

impl<'a, N: Clone, V> Subsimplex<'a, N, V> {
    /// The margin of this subsimplex.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }

    /// Set of vertices containing this subsimplex vertices.
    #[inline]
    pub fn vertices(&self) -> &'a [V] {
        self.vertices
    }

    /// Indices of the vertices of this subsimplex.
    #[inline]
    pub fn indices(&self) -> &'a [uint] {
        self.indices
    }
}

impl<'a,
     N: Clone + Algebraic + Signed + Ord + Bounded,
     V: AlgebraicVecExt<N>,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Subsimplex<'a, N, V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        // XXX: this could be improved _a lot_, computing all support point at once.
        implicit_shape_aabb(m, self)
    }
}

impl<'a, N: Clone, V> HasMargin<N> for Subsimplex<'a, N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl<'a,
     N: Clone + Neg<N> + Ord + Bounded + Algebraic,
     V: AlgebraicVec<N>,
     M: Rotate<V> + Transform<V>>
Implicit<N, V, M> for Subsimplex<'a, N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir    = m.inv_rotate(dir);
        let mut best     = 0;
        let _M: N        = Bounded::max_value();
        let mut best_dot = -_M;

        for i in self.indices.iter() {
            let dot = na::dot(&self.vertices[*i], &local_dir);

            if dot > best_dot {
                best_dot = dot;
                best     = *i;
            }
        }

        m.transform(&self.vertices[best])
    }
}

impl<'a, N, V, M> PreferedSamplingDirections<V, M> for Subsimplex<'a, N, V> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
