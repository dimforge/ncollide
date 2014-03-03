//! Definition of the triangle geometry.

use nalgebra::na;
use geom::mesh::MeshElement;
use math::{Scalar, Vector};

/// A triangle geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Triangle {
    priv margin: Scalar,
    priv a:      Vector,
    priv b:      Vector,
    priv c:      Vector
}

impl Triangle {
    /// Creates a triangle from three points.
    ///
    /// The triangle is created with a default margin of 0.04.
    #[inline]
    pub fn new(a: Vector, b: Vector, c: Vector) -> Triangle {
        Triangle::new_with_margin(a, b, c, na::cast(0.04))
    }

    /// Creates a triangle from three points and a default margin.
    #[inline]
    pub fn new_with_margin(a: Vector, b: Vector, c: Vector, margin: Scalar) -> Triangle {
        assert!(na::dim::<Vector>() > 1);

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
    pub fn a<'a>(&'a self) -> &'a Vector {
        &'a self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Vector {
        &'a self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c<'a>(&'a self) -> &'a Vector {
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
    fn new_with_vertices_and_indices(vs: &[Vector], is: &[uint], margin: Scalar) -> Triangle {
        assert!(is.len() == 3);

        Triangle::new_with_margin(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone(), margin)
    }
}
