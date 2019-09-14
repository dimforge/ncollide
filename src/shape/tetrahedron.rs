//! Definition of the tetrahedron shape.

use crate::math::{Matrix, Point};
use crate::shape::{Segment, Triangle};
use na::RealField;
use std::mem;

/// A tetrahedron with 4 vertices.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Tetrahedron<N: RealField> {
    a: Point<N>,
    b: Point<N>,
    c: Point<N>,
    d: Point<N>,
}

/// Logical description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TetrahedronPointLocation<N: RealField> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on an edge.
    ///
    /// The 0-st edge is the segment AB.
    /// The 1-st edge is the segment AC.
    /// The 2-nd edge is the segment AD.
    /// The 3-rd edge is the segment BC.
    /// The 4-th edge is the segment BD.
    /// The 5-th edge is the segment CD.
    OnEdge(usize, [N; 2]),
    /// The point lies on a triangular face interior.
    ///
    /// The first face is the triangle ABC.
    /// The second face is the triangle ABD.
    /// The third face is the triangle ACD.
    /// The fourth face is the triangle BDC.
    OnFace(usize, [N; 3]),
    /// The point lies inside of the tetrahedron.
    OnSolid,
}

impl<N: RealField> TetrahedronPointLocation<N> {
    /// The barycentric coordinates corresponding to this point location.
    ///
    /// Returns `None` if the location is `TetrahedronPointLocation::OnSolid`.
    pub fn barycentric_coordinates(&self) -> Option<[N; 4]> {
        let mut bcoords = [N::zero(); 4];

        match self {
            TetrahedronPointLocation::OnVertex(i) => bcoords[*i] = N::one(),
            TetrahedronPointLocation::OnEdge(i, uv) => {
                let idx = Tetrahedron::<N>::edge_ids(*i);
                bcoords[idx.0] = uv[0];
                bcoords[idx.1] = uv[1];
            }
            TetrahedronPointLocation::OnFace(i, uvw) => {
                let idx = Tetrahedron::<N>::face_ids(*i);
                bcoords[idx.0] = uvw[0];
                bcoords[idx.1] = uvw[1];
                bcoords[idx.2] = uvw[2];
            }
            TetrahedronPointLocation::OnSolid => {
                return None;
            }
        }

        Some(bcoords)
    }

    /// Returns `true` if both `self` and `other` correspond to points on the same feature of a tetrahedron.
    pub fn same_feature_as(&self, other: &TetrahedronPointLocation<N>) -> bool {
        match (*self, *other) {
            (TetrahedronPointLocation::OnVertex(i), TetrahedronPointLocation::OnVertex(j)) => {
                i == j
            }
            (TetrahedronPointLocation::OnEdge(i, _), TetrahedronPointLocation::OnEdge(j, _)) => {
                i == j
            }
            (TetrahedronPointLocation::OnFace(i, _), TetrahedronPointLocation::OnFace(j, _)) => {
                i == j
            }
            (TetrahedronPointLocation::OnSolid, TetrahedronPointLocation::OnSolid) => true,
            _ => false,
        }
    }
}

impl<N: RealField> Tetrahedron<N> {
    /// Creates a tetrahedron from three points.
    #[inline]
    pub fn new(a: Point<N>, b: Point<N>, c: Point<N>, d: Point<N>) -> Tetrahedron<N> {
        Tetrahedron { a, b, c, d }
    }

    /// Creates the reference to a tetrahedron from the reference to an array of four points.
    pub fn from_array(arr: &[Point<N>; 4]) -> &Tetrahedron<N> {
        unsafe { mem::transmute(arr) }
    }

    /// The fist point of this tetrahedron.
    #[inline]
    pub fn a(&self) -> &Point<N> {
        &self.a
    }

    /// The second point of this tetrahedron.
    #[inline]
    pub fn b(&self) -> &Point<N> {
        &self.b
    }

    /// The third point of this tetrahedron.
    #[inline]
    pub fn c(&self) -> &Point<N> {
        &self.c
    }

    /// The fourth point of this tetrahedron.
    #[inline]
    pub fn d(&self) -> &Point<N> {
        &self.d
    }

    /// Returns the i-th face of this tetrahedron.
    ///
    /// The 0-th face is the triangle ABC.
    /// The 1-st face is the triangle ABD.
    /// The 2-nd face is the triangle ACD.
    /// The 3-rd face is the triangle BCD.
    pub fn face(&self, i: usize) -> Triangle<N> {
        match i {
            0 => Triangle::new(self.a, self.b, self.c),
            1 => Triangle::new(self.a, self.b, self.d),
            2 => Triangle::new(self.a, self.c, self.d),
            3 => Triangle::new(self.b, self.c, self.d),
            _ => panic!("Tetrahedron face index out of bounds (must be < 4."),
        }
    }

    /// Returns the i-th face of this tetrahedron.
    ///
    /// The 0-th face is the triangle ABC.
    /// The 1-st face is the triangle ABD.
    /// The 2-nd face is the triangle ACD.
    /// The 3-rd face is the triangle BCD.
    pub fn face_ids(i: usize) -> (usize, usize, usize) {
        match i {
            0 => (0, 1, 2),
            1 => (0, 1, 3),
            2 => (0, 2, 3),
            3 => (1, 2, 3),
            _ => panic!("Tetrahedron face index out of bounds (must be < 4."),
        }
    }

    /// Returns the i-th edge of this tetrahedron.
    ///
    /// The 0-st edge is the segment AB.
    /// The 1-st edge is the segment AC.
    /// The 2-nd edge is the segment AD.
    /// The 3-rd edge is the segment BC.
    /// The 4-th edge is the segment BD.
    /// The 5-th edge is the segment CD.
    pub fn edge(&self, i: usize) -> Segment<N> {
        match i {
            0 => Segment::new(self.a, self.b),
            1 => Segment::new(self.a, self.c),
            2 => Segment::new(self.a, self.d),
            3 => Segment::new(self.b, self.c),
            4 => Segment::new(self.b, self.d),
            5 => Segment::new(self.c, self.d),
            _ => panic!("Tetrahedron edge index out of bounds (must be < 6)."),
        }
    }

    /// Returns the indices of the vertices of the i-th edge of this tetrahedron.
    ///
    /// The 0-st edge is the segment AB.
    /// The 1-st edge is the segment AC.
    /// The 2-nd edge is the segment AD.
    /// The 3-rd edge is the segment BC.
    /// The 4-th edge is the segment BD.
    /// The 5-th edge is the segment CD.
    pub fn edge_ids(i: usize) -> (usize, usize) {
        match i {
            0 => (0, 1),
            1 => (0, 2),
            2 => (0, 3),
            3 => (1, 2),
            4 => (1, 3),
            5 => (2, 3),
            _ => panic!("Tetrahedron edge index out of bounds (must be < 6)."),
        }
    }

    /// Computes the barycentric coordinates of the given point in the coordinate system of this tetrahedron.
    ///
    /// Returns `None` if this tetrahedron is degenerate.
    pub fn barycentric_coordinates(&self, p: &Point<N>) -> Option<[N; 4]> {
        let ab = self.b - self.a;
        let ac = self.c - self.a;
        let ad = self.d - self.a;
        let m = Matrix::new(ab.x, ac.x, ad.x, ab.y, ac.y, ad.y, ab.z, ac.z, ad.z);

        m.try_inverse().map(|im| {
            let bcoords = im * (p - self.a);
            [
                N::one() - bcoords.x - bcoords.y - bcoords.z,
                bcoords.x,
                bcoords.y,
                bcoords.z,
            ]
        })
    }
}
