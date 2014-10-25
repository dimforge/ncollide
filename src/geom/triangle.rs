//! Definition of the triangle geometry.

use na::Dim;
use na;
use geom::MeshElement;


/// A triangle geometry.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
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

impl<P: Clone + Dim> MeshElement<P> for Triangle<P> {
    #[inline]
    fn nvertices(_: Option<Triangle<P>>) -> uint {
        3
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &[uint]) -> Triangle<P> {
        assert!(is.len() == 3);

        Triangle::new(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone())
    }
}
