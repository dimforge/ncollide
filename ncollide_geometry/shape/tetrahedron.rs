//! Definition of the tetrahedron shape.

use std::mem;
use na;
use shape::{Segment, Triangle};
use math::Point;


/// A tetrahedron with 4 vertices.
#[derive(Copy, Clone, Debug)]
pub struct Tetrahedron<P> {
    a: P,
    b: P,
    c: P,
    d: P
}

impl<P: Point> Tetrahedron<P> {
    /// Creates a tetrahedron from three points.
    #[inline]
    pub fn new(a: P, b: P, c: P, d: P) -> Tetrahedron<P> {
        assert!(na::dimension::<P::Vector>() > 1);
        Tetrahedron { a, b, c, d }
    }

    /// Creates the reference to a tetrahedron from the reference to an array of four points.
    pub fn from_array(arr: &[P; 4]) -> &Tetrahedron<P> {
        unsafe {
            mem::transmute(arr)
        }
    }

    /// The fist point of this tetrahedron.
    #[inline]
    pub fn a(&self) -> &P {
        &self.a
    }

    /// The second point of this tetrahedron.
    #[inline]
    pub fn b(&self) -> &P {
        &self.b
    }

    /// The third point of this tetrahedron.
    #[inline]
    pub fn c(&self) -> &P {
        &self.c
    }

    /// The fourth point of this tetrahedron.
    #[inline]
    pub fn d(&self) -> &P {
        &self.d
    }

    /// Retuns the i-th face of this tetrahedron.
    ///
    /// The 0-th face is the triangle ABC.
    /// The 1-st face is the triangle ABD.
    /// The 2-nd face is the triangle ACD.
    /// The 3-rd face is the triangle BCD.
    pub fn face(&self, i: usize) -> Triangle<P> {
        match i {
            0 => Triangle::new(self.a, self.b, self.c),
            1 => Triangle::new(self.a, self.b, self.d),
            2 => Triangle::new(self.a, self.c, self.d),
            3 => Triangle::new(self.b, self.c, self.d),
            _ => panic!("Tetrahedron face index out of bounds (must be < 4.")
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
    pub fn edge(&self, i: usize) -> Segment<P> {
        match i {
            0 => Segment::new(self.a, self.b),
            1 => Segment::new(self.a, self.c),
            2 => Segment::new(self.a, self.d),
            3 => Segment::new(self.b, self.c),
            4 => Segment::new(self.b, self.d),
            5 => Segment::new(self.c, self.d),
            _ => panic!("Tetrahedron edge index out of bounds (must be < 6).")
        }
    }
}
