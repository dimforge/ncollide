//! Definition of the triangle geometry.

use nalgebra::na;
use geom::mesh::MeshElement;
use math::{Scalar, Vect};

/// A triangle geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Triangle {
    margin: Scalar,
    a:      Vect,
    b:      Vect,
    c:      Vect
}

impl Triangle {
    /// Creates a triangle from three points.
    ///
    /// The triangle is created with a default margin of 0.04.
    #[inline]
    pub fn new(a: Vect, b: Vect, c: Vect) -> Triangle {
        Triangle::new_with_margin(a, b, c, na::cast(0.04))
    }

    /// Creates a triangle from three points and a default margin.
    #[inline]
    pub fn new_with_margin(a: Vect, b: Vect, c: Vect, margin: Scalar) -> Triangle {
        assert!(na::dim::<Vect>() > 1);

        Triangle {
            margin: margin,
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
        &'a self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Vect {
        &'a self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c<'a>(&'a self) -> &'a Vect {
        &'a self.c
    }

    /// The margin surrounding this triangle.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}

impl MeshElement for Triangle {
    #[inline]
    fn nvertices(_: Option<Triangle>) -> uint {
        3
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Vect], is: &[uint], margin: Scalar) -> Triangle {
        assert!(is.len() == 3);

        Triangle::new_with_margin(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone(), margin)
    }
}
