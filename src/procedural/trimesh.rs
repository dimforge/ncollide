use std::collections::HashMap;
use alga::linear::{Rotation, Translation};
use na::{self, Point2, Point3, Real};
use utils::AsBytes;
use super::utils;
use math::{Isometry, Point, Vector};

/// Different representations of the index buffer.
#[derive(Clone, Debug)]
pub enum IndexBuffer {
    /// The vertex, normal, and uvs share the same indices.
    Unified(Vec<Point3<u32>>),
    /// The vertex, normal, and uvs have different indices.
    Split(Vec<Point3<Point3<u32>>>),
}

impl IndexBuffer {
    /// Returns the unified index buffer data or fails.
    #[inline]
    pub fn unwrap_unified(self) -> Vec<Point3<u32>> {
        match self {
            IndexBuffer::Unified(b) => b,
            _ => panic!("Unable to unwrap to an unified buffer."),
        }
    }

    /// Returns the split index buffer data or fails.
    #[inline]
    pub fn unwrap_split(self) -> Vec<Point3<Point3<u32>>> {
        match self {
            IndexBuffer::Split(b) => b,
            _ => panic!("Unable to unwrap to a split buffer."),
        }
    }
}

#[derive(Clone, Debug)]
/// Geometric description of a mesh.
pub struct TriMesh<N: Real> {
    // FIXME: those should *not* be public.
    /// Coordinates of the mesh vertices.
    pub coords: Vec<Point<N>>,
    /// Coordinates of the mesh normals.
    pub normals: Option<Vec<Vector<N>>>,
    /// Textures coordinates of the mesh.
    pub uvs: Option<Vec<Point2<N>>>,
    /// Index buffer of the mesh.
    pub indices: IndexBuffer,
}

impl<N: Real> TriMesh<N> {
    /// Creates a new `TriMesh`.
    ///
    /// If no `indices` is provided, trivial, sequential indices are generated.
    pub fn new(
        coords: Vec<Point<N>>,
        normals: Option<Vec<Vector<N>>>,
        uvs: Option<Vec<Point2<N>>>,
        indices: Option<IndexBuffer>,
    ) -> TriMesh<N> {
        // generate trivial indices
        let idx = indices.unwrap_or_else(|| {
            IndexBuffer::Unified(
                (0..coords.len() / 3)
                    .map(|i| Point3::new(i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2))
                    .collect(),
            )
        });

        TriMesh {
            coords: coords,
            normals: normals,
            uvs: uvs,
            indices: idx,
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
    pub fn translate_by<T: Translation<Point<N>>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform_point(c);
        }
    }

    /// Transforms each vertex and rotates each normal of this mesh.
    #[inline]
    pub fn transform_by(&mut self, t: &Isometry<N>) {
        for c in self.coords.iter_mut() {
            *c = t * &*c;
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t * &*n;
            }
        }
    }

    /// The number of triangles on this mesh.
    #[inline]
    pub fn num_triangles(&self) -> usize {
        match self.indices {
            IndexBuffer::Unified(ref idx) => idx.len(),
            IndexBuffer::Split(ref idx) => idx.len(),
        }
    }

    /// Returns only the vertex ids from the index buffer.
    #[inline]
    pub fn flat_indices(&self) -> Vec<u32> {
        let mut res = Vec::with_capacity(self.num_triangles() * 3);

        match self.indices {
            IndexBuffer::Unified(ref idx) => for i in idx {
                res.push(i[0]);
                res.push(i[1]);
                res.push(i[2]);
            },
            IndexBuffer::Split(ref idx) => for i in idx {
                res.push(i[0][0]);
                res.push(i[1][0]);
                res.push(i[2][0]);
            },
        }

        res
    }
}

impl<N: Real> TriMesh<N> {
    /// Recomputes the mesh normals using its vertex coordinates and adjascency informations
    /// infered from the index buffer.
    #[inline]
    pub fn recompute_normals(&mut self) {
        let mut new_normals = Vec::new();

        match self.indices {
            IndexBuffer::Unified(ref idx) => {
                utils::compute_normals(&self.coords[..], &idx[..], &mut new_normals);
            }
            IndexBuffer::Split(ref idx) => {
                // XXX: too bad we have to reconstruct the index buffer here.
                // The utils::recompute_normals function should be generic wrt. the index buffer
                // type (it could use an iterator instead).
                let coord_idx: Vec<Point3<u32>> = idx.iter()
                    .map(|t| Point3::new(t.x.x, t.y.x, t.z.x))
                    .collect();

                utils::compute_normals(&self.coords[..], &coord_idx[..], &mut new_normals);
            }
        }

        self.normals = Some(new_normals);
    }

    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &Vector<N>) {
        for c in self.coords.iter_mut() {
            for i in 0..na::dimension::<Vector<N>>() {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }
}

impl<N: Real> TriMesh<N> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by_scalar(&mut self, s: N) {
        for c in self.coords.iter_mut() {
            *c = *c * s
        }
    }
}

impl<N: Real> TriMesh<N> {
    // FIXME: looks very similar to the `reformat` on obj.rs
    /// Force the mesh to use the same index for vertices, normals and uvs.
    ///
    /// This might cause the duplication of some vertices, normals and uvs.
    /// Use this method to transform the mesh data to a OpenGL-compliant format.
    pub fn unify_index_buffer(&mut self) {
        let new_indices = match self.indices {
            IndexBuffer::Split(ref ids) => {
                let mut vt2id: HashMap<Point3<u32>, u32> = HashMap::new();
                let mut resi: Vec<u32> = Vec::new();
                let mut resc: Vec<Point<N>> = Vec::new();
                let mut resn: Option<Vec<Vector<N>>> = self.normals.as_ref().map(|_| Vec::new());
                let mut resu: Option<Vec<Point2<N>>> = self.uvs.as_ref().map(|_| Vec::new());

                for triangle in ids.iter() {
                    for point in triangle.iter() {
                        let idx = match vt2id.get(point) {
                            Some(i) => {
                                resi.push(*i);
                                None
                            }
                            None => {
                                let idx = resc.len() as u32;

                                resc.push(self.coords[point.x as usize].clone());

                                let _ = resn.as_mut().map(|l| {
                                    l.push(self.normals.as_ref().unwrap()[point.y as usize].clone())
                                });
                                let _ = resu.as_mut().map(|l| {
                                    l.push(self.uvs.as_ref().unwrap()[point.z as usize].clone())
                                });

                                resi.push(idx);

                                Some(idx)
                            }
                        };

                        let _ = idx.map(|i| vt2id.insert(point.clone(), i));
                    }
                }

                self.coords = resc;
                self.normals = resn;
                self.uvs = resu;

                let mut batched_indices = Vec::new();

                assert!(resi.len() % 3 == 0);
                for f in resi[..].chunks(3) {
                    batched_indices.push(Point3::new(f[0], f[1], f[2]));
                }

                Some(IndexBuffer::Unified(batched_indices))
            }
            _ => None,
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }

    /// Unifies the index buffer and ensure duplicate each vertex
    /// are duplicated such that no two vertex entry of the index buffer
    /// are equal.
    pub fn replicate_vertices(&mut self) {
        let mut resi: Vec<u32> = Vec::new();
        let mut resc: Vec<Point<N>> = Vec::new();
        let mut resn: Option<Vec<Vector<N>>> = self.normals.as_ref().map(|_| Vec::new());
        let mut resu: Option<Vec<Point2<N>>> = self.uvs.as_ref().map(|_| Vec::new());

        match self.indices {
            IndexBuffer::Split(ref ids) => for triangle in ids.iter() {
                for point in triangle.iter() {
                    let idx = resc.len() as u32;
                    resc.push(self.coords[point.x as usize].clone());

                    let _ = resn.as_mut()
                        .map(|l| l.push(self.normals.as_ref().unwrap()[point.y as usize].clone()));
                    let _ = resu.as_mut()
                        .map(|l| l.push(self.uvs.as_ref().unwrap()[point.z as usize].clone()));

                    resi.push(idx);
                }
            },
            IndexBuffer::Unified(ref ids) => for triangle in ids.iter() {
                for point in triangle.iter() {
                    let idx = resc.len() as u32;
                    resc.push(self.coords[*point as usize].clone());

                    let _ = resn.as_mut()
                        .map(|l| l.push(self.normals.as_ref().unwrap()[*point as usize].clone()));
                    let _ = resu.as_mut()
                        .map(|l| l.push(self.uvs.as_ref().unwrap()[*point as usize].clone()));

                    resi.push(idx);
                }
            },
        };

        self.coords = resc;
        self.normals = resn;
        self.uvs = resu;

        let mut batched_indices = Vec::new();

        assert!(resi.len() % 3 == 0);
        for f in resi[..].chunks(3) {
            batched_indices.push(Point3::new(f[0], f[1], f[2]));
        }

        self.indices = IndexBuffer::Unified(batched_indices)
    }
}

impl<N: Real> TriMesh<N> {
    /// Forces the mesh to use a different index for the vertices, normals and uvs.
    ///
    /// If `recover_topology` is true, this will merge exactly identical vertices together.
    pub fn split_index_buffer(&mut self, recover_topology: bool) {
        let new_indices = match self.indices {
            IndexBuffer::Unified(ref ids) => {
                let resi;

                if recover_topology {
                    let (idx, coords) =
                        utils::split_index_buffer_and_recover_topology(&ids[..], &self.coords[..]);
                    self.coords = coords;
                    resi = idx;
                } else {
                    resi = utils::split_index_buffer(&ids[..]);
                }

                Some(IndexBuffer::Split(resi))
            }
            _ => None,
        };

        let _ = new_indices.map(|nids| self.indices = nids);
    }
}
