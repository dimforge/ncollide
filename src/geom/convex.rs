//! 
//! Support mapping based Convex polytope.
//!
use math::{Scalar, Vect};

#[cfg(not(feature = "4d"))]
use procedural;

#[cfg(not(feature = "2d"))]
use procedural::TriMesh;

#[cfg(feature = "3d")]
use na::Vec3;

#[cfg(feature = "2d")]
use procedural::Polyline;

#[cfg(feature = "3d")]
use procedural::{UnifiedIndexBuffer, SplitIndexBuffer};

/// Set of point assumed to form a convex polytope.
#[cfg(not(feature = "2d"))]
pub struct Convex {
    mesh:   TriMesh<Scalar, Vect>,
}

/// Set of point assumed to form a convex polyline.
#[cfg(feature = "2d")]
pub struct Convex {
    mesh:   Polyline<Scalar, Vect>,
}

impl Clone for Convex {
    fn clone(&self) -> Convex {
        Convex {
            mesh: self.mesh.clone(),
        }
    }
}

#[cfg(feature = "3d")]
impl Convex {
    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new(points: &[Vect]) -> Convex {
        unsafe {
            Convex::new_with_convex_mesh(procedural::convex_hull3d(points))
        }
    }

    /// Creates a polytope from a convex mesh. The convexity is __not__ checked.
    #[inline]
    pub unsafe fn new_with_convex_mesh(mesh: TriMesh<Scalar, Vect>) -> Convex {
        let mut mesh = mesh;

        /*
         * Drop the normals and uvs.
         */
        mesh.normals = None;
        mesh.uvs     = None;

        /*
         * Unify the index buffer.
         */
        let new_index_buffer = 
            match mesh.indices {
                UnifiedIndexBuffer(_) => None,
                SplitIndexBuffer(ref idx) => {
                    let mut buf = Vec::with_capacity(idx.len());

                    for t in idx.iter() {
                        buf.push(Vec3::new(t.x.x, t.y.x, t.z.x))
                    }

                    Some(buf)
                }
            };

        match new_index_buffer {
            Some(idx) => mesh.indices = UnifiedIndexBuffer(idx),
            None      => { }
        }

        assert!(mesh.coords.len() > 0, "A convex geometry must have at least one vertex.");

        Convex {
            mesh: mesh
        }
    }

    /// The convex mesh of this geometry.
    #[inline]
    pub fn mesh<'a>(&'a self) -> &'a TriMesh<Scalar, Vect> {
        &self.mesh
    }

    /// The mutable convex mesh of this geometry.
    #[inline]
    pub fn mesh_mut<'a>(&'a mut self) -> &'a mut TriMesh<Scalar, Vect> {
        &mut self.mesh
    }

    /// The convex mesh of this geometry.
    #[inline]
    pub fn unwrap(self) -> TriMesh<Scalar, Vect> {
        self.mesh
    }
}

#[cfg(feature = "2d")]
impl Convex {
    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new(points: &[Vect]) -> Convex {
        unsafe {
            Convex::new_with_convex_polyline(procedural::convex_hull2d(points))
        }
    }

    /// Creates a polytope from a convex polyline. The convexity is __not__ checked.
    #[inline]
    pub unsafe fn new_with_convex_polyline(mesh: Polyline<Scalar, Vect>) -> Convex {
        let mut mesh = mesh;

        /*
         * Drop the normals and uvs.
         */
        mesh.normals = None;

        Convex {
            mesh: mesh
        }
    }

    /// The convex polyline of this geometry.
    #[inline]
    pub fn mesh<'a>(&'a self) -> &'a Polyline<Scalar, Vect> {
        &self.mesh
    }

    /// The mutable convex polyline of this geometry.
    #[inline]
    pub fn mesh_mut<'a>(&'a mut self) -> &'a mut Polyline<Scalar, Vect> {
        &mut self.mesh
    }

    /// The convex polyline of this geometry.
    #[inline]
    pub fn unwrap(self) -> Polyline<Scalar, Vect> {
        self.mesh
    }
}

impl Convex {
    /// The list of points of this convex polytope.
    #[inline]
    pub fn pts<'a>(&'a self) -> &'a [Vect] { // FIXME: naming: `pts` vs. `points`?
        self.mesh.coords.as_slice()
    }
}
