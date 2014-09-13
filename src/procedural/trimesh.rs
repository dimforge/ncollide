use std::collections::HashMap;
use std::hash::Hash;
use nalgebra::na::{Indexable, Dim, Iterable, Translate, Rotate, Transform, FloatVec, Cross, Vec3, Vec2};
use nalgebra::na;
use procedural::utils;
use utils::AsBytes;

/// Different representations of the index buffer.
#[deriving(Clone, Show)]
pub enum IndexBuffer {
    /// The vertex, normal, and uvs share the same indices.
    UnifiedIndexBuffer(Vec<Vec3<u32>>),
    /// The vertex, normal, and uvs have different indices.
    SplitIndexBuffer(Vec<Vec3<Vec3<u32>>>)
}

impl IndexBuffer {
    /// Returns the unified index buffer data or fails.
    #[inline]
    pub fn unwrap_unified(self) -> Vec<Vec3<u32>> {
        match self {
            UnifiedIndexBuffer(b) => b,
            _                     => fail!("Unable to unwrap to an unified buffer.")
        }
    }

    /// Returns the split index buffer data or fails.
    #[inline]
    pub fn unwrap_split(self) -> Vec<Vec3<Vec3<u32>>> {
        match self {
            SplitIndexBuffer(b) => b,
            _                   => fail!("Unable to unwrap to a split buffer.")
        }
    }
}

#[deriving(Clone, Show)]
/// Geometric description of a mesh.
pub struct TriMesh<N, V> {
    // FIXME: those should *not* be public.
    /// Coordinates of the mesh vertices.
    pub coords:  Vec<V>,
    /// Coordinates of the mesh normals.
    pub normals: Option<Vec<V>>,
    /// Textures coordinates of the mesh.
    pub uvs:     Option<Vec<Vec2<N>>>,
    /// Index buffer of the mesh.
    pub indices: IndexBuffer
}

impl<N, V> TriMesh<N, V> {
    /// Creates a new `TriMesh`.
    ///
    /// If no `indices` is provided, trivial, sequential indices are generated.
    pub fn new(coords:  Vec<V>,
               normals: Option<Vec<V>>,
               uvs:     Option<Vec<Vec2<N>>>,
               indices: Option<IndexBuffer>)
               -> TriMesh<N, V> {
        // generate trivial indices
        let idx = indices.unwrap_or_else(||
           UnifiedIndexBuffer(
               Vec::from_fn(coords.len() / 3, |i| Vec3::new(i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2))
           )
        );

        TriMesh {
            coords:  coords,
            normals: normals,
            uvs:     uvs,
            indices: idx
        }
    }

    /// Whether or not this triangle mesh has normals.
    #[inline]
    pub fn has_normals(&self) -> bool {
        self.normals.is_some()
    }

    /// Whether or not this triangle mesh has texture coordinates.
    #[inline]
    pub fn has_uvs(&self) -> bool {
        self.uvs.is_some()
    }

    /// Translates each vertex of this mesh.
    #[inline]
    pub fn translate_by<T: Translate<V>>(&mut self, t: &T) {
        for c in self.coords.mut_iter() {
            *c = t.translate(c);
        }
    }

    /// Rotates each vertex and normal of this mesh.
    #[inline]
    pub fn rotate_by<R: Rotate<V>>(&mut self, r: &R) {
        for c in self.coords.mut_iter() {
            *c = r.rotate(c);
        }

        for n in self.normals.mut_iter() {
            for n in n.mut_iter() {
                *n = r.rotate(n);
            }
        }
    }

    /// Transforms each vertex and rotates each normal of this mesh.
    #[inline]
    pub fn transform_by<T: Transform<V> + Rotate<V>>(&mut self, t: &T) {
        for c in self.coords.mut_iter() {
            *c = t.transform(c);
        }

        for n in self.normals.mut_iter() {
            for n in n.mut_iter() {
                *n = t.rotate(n);
            }
        }
    }

    /// The number of triangles on this mesh.
    #[inline]
    pub fn num_triangles(&self) -> uint {
        match self.indices {
            UnifiedIndexBuffer(ref idx) => idx.len(),
            SplitIndexBuffer(ref idx)   => idx.len()
        }
    }
}

impl<N: Float, V: FloatVec<N> + Cross<V> + Clone> TriMesh<N, V> {
    /// Recomputes the mesh normals using its vertex coordinates and adjascency informations
    /// infered from the index buffer.
    #[inline]
    pub fn recompute_normals(&mut self) {
        let mut new_normals = Vec::new();

        match self.indices {
            UnifiedIndexBuffer(ref idx) => {
                utils::compute_normals(self.coords.as_slice(),
                                       idx.as_slice(),
                                       &mut new_normals);
            },
            SplitIndexBuffer(ref idx) => {
                // XXX: too bad we have to reconstruct the index buffer here.
                // The utils::recompute_normals function should be generic wrt. the index buffer
                // type (it could use an iterator instead).
                let coord_idx: Vec<Vec3<u32>> = idx.iter().map(|t| Vec3::new(t.x.x, t.y.x, t.z.x)).collect();

                utils::compute_normals(self.coords.as_slice(),
                                       coord_idx.as_slice(),
                                       &mut new_normals);
            }
        }

        self.normals = Some(new_normals);
    }
}

impl<N: Mul<N, N>, V: Dim + Indexable<uint, N>> TriMesh<N, V> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &V) {
        for c in self.coords.mut_iter() {
            for i in range(0, na::dim::<V>()) {
                let val = c.at(i);
                let mul = s.at(i);
                c.set(i, val * mul);
            }
        }
        // FIXME: do something for the normals?
    }
}

impl<N, V: Mul<N, V>> TriMesh<N, V> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by_scalar(&mut self, s: &N) {
        for c in self.coords.mut_iter() {
            *c = *c * *s
        }
    }
}

impl<N: Clone, V: Clone> TriMesh<N, V> {
    // FIXME: looks very similar to the `reformat` on obj.rs
    /// Force the mesh to use the same index for vertices, normals and uvs.
    ///
    /// This might cause the duplication of some vertices, normals and uvs.
    /// Use this method to transform the mesh data to a OpenGL-compliant format.
    pub fn unify_index_buffer(&mut self) {
        let new_indices = match self.indices {
            SplitIndexBuffer(ref ids) => {
                let mut vt2id:HashMap<Vec3<u32>, u32> = HashMap::new();
                let mut resi: Vec<u32>                = Vec::new();
                let mut resc: Vec<V>                  = Vec::new();
                let mut resn: Option<Vec<V>>          = self.normals.as_ref().map(|_| Vec::new());
                let mut resu: Option<Vec<Vec2<N>>>    = self.uvs.as_ref().map(|_| Vec::new());

                for triangle in ids.iter() {
                    for point in triangle.iter() {
                        let idx = match vt2id.find(point) {
                            Some(i) => { resi.push(*i); None },
                            None    => {
                                let idx = resc.len() as u32;

                                resc.push(self.coords[point.x as uint].clone());

                                let _ = resn.as_mut().map(|l| l.push(self.normals.as_ref().unwrap()[point.y as uint].clone()));
                                let _ = resu.as_mut().map(|l| l.push(self.uvs.as_ref().unwrap()[point.z as uint].clone()));

                                resi.push(idx);

                                Some(idx)
                            }
                        };

                        let _ = idx.map(|i| vt2id.insert(point.clone(), i));
                    }
                }

                self.coords  = resc;
                self.normals = resn;
                self.uvs     = resu;

                let mut batched_indices = Vec::new();

                assert!(resi.len() % 3 == 0);
                for f in resi.as_slice().chunks(3) {
                    batched_indices.push(Vec3::new(f[0], f[1], f[2]));
                }

                Some(UnifiedIndexBuffer(batched_indices))
            }
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}

impl<N: Clone, V: Clone + PartialEq + AsBytes> TriMesh<N, V> {
    /// Forces the mesh to use a different index for the vertices, normals and uvs.
    ///
    /// If `recover_topology` is true, this will merge exactly identical vertices together.
    pub fn split_index_buffer(&mut self, recover_topology: bool) {
        let new_indices = match self.indices {
            UnifiedIndexBuffer(ref ids) => {
                let resi;

                if recover_topology {
                    let (idx, coords) = utils::split_index_buffer_and_recover_topology(
                                           ids.as_slice(),
                                           self.coords.as_slice());
                    self.coords = coords;
                    resi = idx;
                }
                else {
                    resi = utils::split_index_buffer(ids.as_slice());
                }

                Some(SplitIndexBuffer(resi))
            },
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}
