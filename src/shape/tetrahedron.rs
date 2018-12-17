//! Definition of the tetrahedron shape.

use std::mem;
use na::Real;
use shape::{Segment, Triangle};
use math::Point;

/// A tetrahedron with 4 vertices.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Tetrahedron<N: Real> {
    a: Point<N>,
    b: Point<N>,
    c: Point<N>,
    d: Point<N>,
}

/// Logical description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TetrahedronPointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on a vertex.
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

impl<N: Real> TetrahedronPointLocation<N> {
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

impl<N: Real> Tetrahedron<N> {
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

    /// Retuns the i-th face of this tetrahedron.
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

    /// Retuns the i-th edge of this tetrahedron.
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
}
