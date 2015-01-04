use std::ops::Mul;
use std::ops::Index;
use std::ops::IndexMut;
use std::collections::HashMap;
use na::{Dim, Iterable, Translate, Rotate, Transform, Cross, Vec3, Pnt2};
use na;
use procedural::utils;
use utils::AsBytes;
use math::{Scalar, Point, Vect};

/// Different representations of the index buffer.
#[derive(Clone, Show)]
pub enum IndexBuffer {
    /// The vertex, normal, and uvs share the same indices.
    Unified(Vec<Vec3<u32>>),
    /// The vertex, normal, and uvs have different indices.
    Split(Vec<Vec3<Vec3<u32>>>)
}

impl IndexBuffer {
    /// Returns the unified index buffer data or fails.
    #[inline]
    pub fn unwrap_unified(self) -> Vec<Vec3<u32>> {
        match self {
            IndexBuffer::Unified(b) => b,
            _ => panic!("Unable to unwrap to an unified buffer.")
        }
    }

    /// Returns the split index buffer data or fails.
    #[inline]
    pub fn unwrap_split(self) -> Vec<Vec3<Vec3<u32>>> {
        match self {
            IndexBuffer::Split(b) => b,
            _ => panic!("Unable to unwrap to a split buffer.")
        }
    }
}

#[derive(Clone, Show)]
/// Shapeetric description of a mesh.
pub struct TriMesh<N, P, V> {
    // FIXME: those should *not* be public.
    /// Coordinates of the mesh vertices.
    pub coords:  Vec<P>,
    /// Coordinates of the mesh normals.
    pub normals: Option<Vec<V>>,
    /// Textures coordinates of the mesh.
    pub uvs:     Option<Vec<Pnt2<N>>>,
    /// Index buffer of the mesh.
    pub indices: IndexBuffer
}

impl<N, P, V> TriMesh<N, P, V> {
    /// Creates a new `TriMesh`.
    ///
    /// If no `indices` is provided, trivial, sequential indices are generated.
    pub fn new(coords:  Vec<P>,
               normals: Option<Vec<V>>,
               uvs:     Option<Vec<Pnt2<N>>>,
               indices: Option<IndexBuffer>)
               -> TriMesh<N, P, V> {
        // generate trivial indices
        let idx = indices.unwrap_or_else(||
           IndexBuffer::Unified(
               range(0, coords.len() / 3)
                .map(|i| Vec3::new(i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2))
                .collect()
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
    pub fn translate_by<T: Translate<P>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.translate(c);
        }
    }

    /// Transforms each vertex and rotates each normal of this mesh.
    #[inline]
    pub fn transform_by<T: Transform<P> + Rotate<V>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform(c);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t.rotate(n);
            }
        }
    }

    /// The number of triangles on this mesh.
    #[inline]
    pub fn num_triangles(&self) -> uint {
        match self.indices {
            IndexBuffer::Unified(ref idx) => idx.len(),
            IndexBuffer::Split(ref idx)   => idx.len()
        }
    }
}

impl<N, P: Point<N, V>, V> TriMesh<N, P, V> {
    /// Rotates each vertex and normal of this mesh.
    #[inline]
    // XXX: we should use Rotate<P> instead of the .set_coord.
    // Wa cannot make it a Rotate because the `Rotate` bound cannot appear twice… we have, again,
    // to wait for the trait reform.
    pub fn rotate_by<R: Rotate<V>>(&mut self, r: &R) {
        for c in self.coords.iter_mut() {
            let rc = r.rotate(c.as_vec());
            c.set_coords(rc);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = r.rotate(n);
            }
        }
    }
}

impl<N, P, V> TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Cross<V> {
    /// Recomputes the mesh normals using its vertex coordinates and adjascency informations
    /// infered from the index buffer.
    #[inline]
    pub fn recompute_normals(&mut self) {
        let mut new_normals = Vec::new();

        match self.indices {
            IndexBuffer::Unified(ref idx) => {
                utils::compute_normals(self.coords.as_slice(), idx.as_slice(), &mut new_normals);
            },
            IndexBuffer::Split(ref idx) => {
                // XXX: too bad we have to reconstruct the index buffer here.
                // The utils::recompute_normals function should be generic wrt. the index buffer
                // type (it could use an iterator instead).
                let coord_idx: Vec<Vec3<u32>> = idx.iter().map(|t| Vec3::new(t.x.x, t.y.x, t.z.x)).collect();

                utils::compute_normals(self.coords.as_slice(), coord_idx.as_slice(), &mut new_normals);
            }
        }

        self.normals = Some(new_normals);
    }
}

impl<N, P, V> TriMesh<N, P, V>
    where N: Scalar,
          P: Index<uint, N> + IndexMut<uint, N>,
          V: Dim + Index<uint, N> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &V) {
        for c in self.coords.iter_mut() {
            for i in range(0, na::dim::<V>()) {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }
}

impl<N: Copy, P: Copy + Mul<N, P>, V> TriMesh<N, P, V> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by_scalar(&mut self, s: N) {
        for c in self.coords.iter_mut() {
            *c = *c * s
        }
    }
}

impl<N: Clone, P: Clone, V: Clone> TriMesh<N, P, V> {
    // FIXME: looks very similar to the `reformat` on obj.rs
    /// Force the mesh to use the same index for vertices, normals and uvs.
    ///
    /// This might cause the duplication of some vertices, normals and uvs.
    /// Use this method to transform the mesh data to a OpenGL-compliant format.
    pub fn unify_index_buffer(&mut self) {
        let new_indices = match self.indices {
            IndexBuffer::Split(ref ids) => {
                let mut vt2id:HashMap<Vec3<u32>, u32> = HashMap::new();
                let mut resi: Vec<u32>                = Vec::new();
                let mut resc: Vec<P>                  = Vec::new();
                let mut resn: Option<Vec<V>>          = self.normals.as_ref().map(|_| Vec::new());
                let mut resu: Option<Vec<Pnt2<N>>>    = self.uvs.as_ref().map(|_| Vec::new());

                for triangle in ids.iter() {
                    for point in triangle.iter() {
                        let idx = match vt2id.get(point) {
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

                Some(IndexBuffer::Unified(batched_indices))
            }
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}

impl<N, P, V> TriMesh<N, P, V>
    where N: Scalar,
          P: AsBytes + PartialEq + Clone {
    /// Forces the mesh to use a different index for the vertices, normals and uvs.
    ///
    /// If `recover_topology` is true, this will merge exactly identical vertices together.
    pub fn split_index_buffer(&mut self, recover_topology: bool) {
        let new_indices = match self.indices {
            IndexBuffer::Unified(ref ids) => {
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

                Some(IndexBuffer::Split(resi))
            },
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}
