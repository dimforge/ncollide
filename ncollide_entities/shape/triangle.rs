//! Definition of the triangle shape.

use na::{Dim, Pnt3};
use na;
use shape::BaseMeshElement;


/// A triangle shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Triangle<P> {
    a: P,
    b: P,
    c: P
}

impl<P: Dim> Triangle<P> {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: P, b: P, c: P) -> Triangle<P> {
        assert!(na::dim::<P>() > 1);

        Triangle {
            a: a,
            b: b,
            c: c
        }
    }
}

impl<P> Triangle<P> {
    /// The fist point of this triangle.
    #[inline]
    pub fn a(&self) -> &P {
        &self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b(&self) -> &P {
        &self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c(&self) -> &P {
        &self.c
    }
}

impl<P: Copy + Dim> BaseMeshElement<Pnt3<u32>, P> for Triangle<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Pnt3<u32>) -> Triangle<P> {
        Triangle::new(vs[is.x as usize], vs[is.y as usize], vs[is.z as usize])
    }
}
