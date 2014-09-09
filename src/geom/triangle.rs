//! Definition of the triangle geometry.

use nalgebra::na;
use geom::mesh::MeshElement;
use math::{Scalar, Vect};

/// A triangle geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Triangle {
    a:      Vect,
    b:      Vect,
    c:      Vect
}

impl Triangle {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: Vect, b: Vect, c: Vect) -> Triangle {
        assert!(na::dim::<Vect>() > 1);

        Triangle {
            a:      a,
            b:      b,
            c:      c
        }
    }
}

impl Triangle {
    /// The fist point of this triangle.
    #[inline]
    pub fn a<'a>(&'a self) -> &'a Vect {
        &self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Vect {
        &self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c<'a>(&'a self) -> &'a Vect {
        &self.c
    }
}

impl MeshElement for Triangle {
    #[inline]
    fn nvertices(_: Option<Triangle>) -> uint {
        3
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Vect], is: &[uint]) -> Triangle {
        assert!(is.len() == 3);

        Triangle::new(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone())
    }
}
