use std::collections::HashMap;
use na::{Iterable, Translate, Rotate, Transform, Cross, Pnt2, Pnt3};
use na;
use super::utils;
use ncollide_utils::AsBytes;
use math::{Point, Vect};

/// Different representations of the index buffer.
#[derive(Clone, Debug)]
pub enum IndexBuffer {
    /// The vertex, normal, and uvs share the same indices.
    Unified(Vec<Pnt3<u32>>),
    /// The vertex, normal, and uvs have different indices.
    Split(Vec<Pnt3<Pnt3<u32>>>)
}

impl IndexBuffer {
    /// Returns the unified index buffer data or fails.
    #[inline]
    pub fn unwrap_unified(self) -> Vec<Pnt3<u32>> {
        match self {
            IndexBuffer::Unified(b) => b,
            _ => panic!("Unable to unwrap to an unified buffer.")
        }
    }

    /// Returns the split index buffer data or fails.
    #[inline]
    pub fn unwrap_split(self) -> Vec<Pnt3<Pnt3<u32>>> {
        match self {
            IndexBuffer::Split(b) => b,
            _ => panic!("Unable to unwrap to a split buffer.")
        }
    }
}

#[derive(Clone, Debug)]
/// Shapeetric description of a mesh.
pub struct TriMesh<P: Point> {
    // FIXME: those should *not* be public.
    /// Coordinates of the mesh vertices.
    pub coords:  Vec<P>,
    /// Coordinates of the mesh normals.
    pub normals: Option<Vec<P::Vect>>,
    /// Textures coordinates of the mesh.
    pub uvs:     Option<Vec<Pnt2<<P::Vect as Vect>::Scalar>>>,
    /// Index buffer of the mesh.
    pub indices: IndexBuffer
}

impl<P: Point> TriMesh<P> {
    /// Creates a new `TriMesh`.
    ///
    /// If no `indices` is provided, trivial, sequential indices are generated.
    pub fn new(coords:  Vec<P>,
               normals: Option<Vec<P::Vect>>,
               uvs:     Option<Vec<Pnt2<<P::Vect as Vect>::Scalar>>>,
               indices: Option<IndexBuffer>)
               -> TriMesh<P> {
        // generate trivial indices
        let idx = indices.unwrap_or_else(||
           IndexBuffer::Unified(
               (0 .. coords.len() / 3).map(|i| Pnt3::new(i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2)).collect()
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
    pub fn transform_by<T: Transform<P> + Rotate<P::Vect>>(&mut self, t: &T) {
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
    pub fn num_triangles(&self) -> usize {
        match self.indices {
            IndexBuffer::Unified(ref idx) => idx.len(),
            IndexBuffer::Split(ref idx)   => idx.len()
        }
    }
}

impl<P: Point> TriMesh<P> {
    /// Rotates each vertex and normal of this mesh.
    #[inline]
    // XXX: we should use Rotate<P> instead of the .set_coord.
    // Wa cannot make it a Rotate because the `Rotate` bound cannot appear twice… we have, again,
    // to wait for the trait reform.
    pub fn rotate_by<R: Rotate<P::Vect>>(&mut self, r: &R) {
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

impl<P> TriMesh<P>
    where P: Point,
          P::Vect: Cross<CrossProductType = <P as Point>::Vect> {
    /// Recomputes the mesh normals using its vertex coordinates and adjascency informations
    /// infered from the index buffer.
    #[inline]
    pub fn recompute_normals(&mut self) {
        let mut new_normals = Vec::new();

        match self.indices {
            IndexBuffer::Unified(ref idx) => {
                utils::compute_normals(&self.coords[..], &idx[..], &mut new_normals);
            },
            IndexBuffer::Split(ref idx) => {
                // XXX: too bad we have to reconstruct the index buffer here.
                // The utils::recompute_normals function should be generic wrt. the index buffer
                // type (it could use an iterator instead).
                let coord_idx: Vec<Pnt3<u32>> = idx.iter().map(|t| Pnt3::new(t.x.x, t.y.x, t.z.x)).collect();

                utils::compute_normals(&self.coords[..], &coord_idx[..], &mut new_normals);
            }
        }

        self.normals = Some(new_normals);
    }
}

impl<P> TriMesh<P>
    where P: Point {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &P::Vect) {
        for c in self.coords.iter_mut() {
            for i in 0 .. na::dim::<P::Vect>() {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }
}

impl<P: Point> TriMesh<P> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by_scalar(&mut self, s: <P::Vect as Vect>::Scalar) {
        for c in self.coords.iter_mut() {
            *c = *c * s
        }
    }
}

impl<P: Point> TriMesh<P> {
    // FIXME: looks very similar to the `reformat` on obj.rs
    /// Force the mesh to use the same index for vertices, normals and uvs.
    ///
    /// This might cause the duplication of some vertices, normals and uvs.
    /// Use this method to transform the mesh data to a OpenGL-compliant format.
    pub fn unify_index_buffer(&mut self) {
        let new_indices = match self.indices {
            IndexBuffer::Split(ref ids) => {
                let mut vt2id:HashMap<Pnt3<u32>, u32> = HashMap::new();
                let mut resi: Vec<u32>                = Vec::new();
                let mut resc: Vec<P>                  = Vec::new();
                let mut resn: Option<Vec<P::Vect>>    = self.normals.as_ref().map(|_| Vec::new());
                let mut resu: Option<Vec<Pnt2<<P::Vect as Vect>::Scalar>>> = self.uvs.as_ref().map(|_| Vec::new());

                for triangle in ids.iter() {
                    for point in triangle.iter() {
                        let idx = match vt2id.get(point) {
                            Some(i) => { resi.push(*i); None },
                            None    => {
                                let idx = resc.len() as u32;

                                resc.push(self.coords[point.x as usize].clone());

                                let _ = resn.as_mut().map(|l| l.push(self.normals.as_ref().unwrap()[point.y as usize].clone()));
                                let _ = resu.as_mut().map(|l| l.push(self.uvs.as_ref().unwrap()[point.z as usize].clone()));

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
                for f in resi[..].chunks(3) {
                    batched_indices.push(Pnt3::new(f[0], f[1], f[2]));
                }

                Some(IndexBuffer::Unified(batched_indices))
            }
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}

impl<P> TriMesh<P>
    where P: Point + AsBytes {
    /// Forces the mesh to use a different index for the vertices, normals and uvs.
    ///
    /// If `recover_topology` is true, this will merge exactly identical vertices together.
    pub fn split_index_buffer(&mut self, recover_topology: bool) {
        let new_indices = match self.indices {
            IndexBuffer::Unified(ref ids) => {
                let resi;

                if recover_topology {
                    let (idx, coords) = utils::split_index_buffer_and_recover_topology(
                                           &ids[..],
                                           &self.coords[..]);
                    self.coords = coords;
                    resi = idx;
                }
                else {
                    resi = utils::split_index_buffer(&ids[..]);
                }

                Some(IndexBuffer::Split(resi))
            },
            _ => None
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}
