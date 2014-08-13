//! 
//! Support mapping based Convex polytope.
//!
use nalgebra::na::Vec3;
use nalgebra::na;
use math::{Scalar, Vect};
use procedural::{Polyline, TriMesh, UnifiedIndexBuffer, SplitIndexBuffer};
use procedural;

/// Set of point assumed to form a convex polytope.
#[not_dim2]
pub struct Convex {
    mesh:   TriMesh<Scalar, Vect>,
    margin: Scalar
}

/// Set of point assumed to form a convex polyline.
#[dim2]
pub struct Convex {
    mesh:   Polyline<Scalar, Vect>,
    margin: Scalar
}

impl Clone for Convex {
    fn clone(&self) -> Convex {
        Convex {
            mesh:   self.mesh.clone(),
            margin: self.margin.clone()
        }
    }
}

#[dim3]
impl Convex {
    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new(points: &[Vect]) -> Convex {
        Convex::new_with_margin(points, na::cast(0.04f64))
    }

    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new_with_margin(points: &[Vect], margin: Scalar) -> Convex {
        unsafe {
            Convex::new_with_convex_mesh(procedural::convex_hull3d(points), margin)
        }
    }

    /// Creates a polytope from a convex mesh. The convexity is __not__ checked.
    #[inline]
    pub unsafe fn new_with_convex_mesh(mesh: TriMesh<Scalar, Vect>, margin: Scalar) -> Convex {
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

        Convex {
            mesh:   mesh,
            margin: margin
        }
    }

    /// The convex mesh of this geometry.
    #[inline]
    pub fn mesh<'a>(&'a self) -> &'a TriMesh<Scalar, Vect> {
        &self.mesh
    }

    /// The convex mesh of this geometry.
    #[inline]
    pub fn unwrap(self) -> TriMesh<Scalar, Vect> {
        self.mesh
    }
}

#[dim2]
impl Convex {
    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new(points: &[Vect]) -> Convex {
        Convex::new_with_margin(points, na::cast(0.04f64))
    }

    /// Creates a polytope from a set of point.
    ///
    /// This computes the convex hull of the set of points internally.
    #[inline]
    pub fn new_with_margin(points: &[Vect], margin: Scalar) -> Convex {
        unsafe {
            Convex::new_with_convex_polyline(procedural::convex_hull2d(points), margin)
        }
    }

    /// Creates a polytope from a convex polyline. The convexity is __not__ checked.
    #[inline]
    pub unsafe fn new_with_convex_polyline(mesh: Polyline<Scalar, Vect>, margin: Scalar) -> Convex {
        let mut mesh = mesh;

        /*
         * Drop the normals and uvs.
         */
        mesh.normals = None;

        Convex {
            mesh:   mesh,
            margin: margin
        }
    }

    /// The convex polyline of this geometry.
    #[inline]
    pub fn mesh<'a>(&'a self) -> &'a Polyline<Scalar, Vect> {
        &self.mesh
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

    /// The margin surrounding this convex polytope.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
