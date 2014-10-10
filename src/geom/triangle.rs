//! Definition of the triangle geometry.

use na;
use geom::mesh::MeshElement;
use math::Point;

/// A triangle geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Triangle {
    a:      Point,
    b:      Point,
    c:      Point
}

impl Triangle {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: Point, b: Point, c: Point) -> Triangle {
        assert!(na::dim::<Point>() > 1);

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
    pub fn a<'a>(&'a self) -> &'a Point {
        &self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Point {
        &self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c<'a>(&'a self) -> &'a Point {
        &self.c
    }
}

impl MeshElement for Triangle {
    #[inline]
    fn nvertices(_: Option<Triangle>) -> uint {
        3
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Point], is: &[uint]) -> Triangle {
        assert!(is.len() == 3);

        Triangle::new(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone())
    }
}
