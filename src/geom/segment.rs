//! Definition of the segment geometry.

use na;
use geom::mesh::MeshElement;
use math::Point;

/// A segment geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Segment {
    a:      Point,
    b:      Point
}

impl Segment {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: Point, b: Point) -> Segment {
        assert!(na::dim::<Point>() > 1);

        Segment {
            a:      a,
            b:      b
        }
    }
}

impl Segment {
    /// The first point of this segment.
    #[inline]
    pub fn a<'a>(&'a self) -> &'a Point {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Point {
        &self.b
    }
}

impl MeshElement for Segment {
    #[inline]
    fn nvertices(_: Option<Segment>) -> uint {
        2
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Point], is: &[uint]) -> Segment {
        assert!(is.len() == 2);

        Segment::new(vs[is[0]].clone(), vs[is[1]].clone())
    }
}
