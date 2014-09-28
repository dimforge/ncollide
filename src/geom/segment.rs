//! Definition of the segment geometry.

use na;
use geom::mesh::MeshElement;
use math::{Scalar, Vect};

/// A segment geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Segment {
    a:      Vect,
    b:      Vect
}

impl Segment {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: Vect, b: Vect) -> Segment {
        assert!(na::dim::<Vect>() > 1);

        Segment {
            a:      a,
            b:      b
        }
    }
}

impl Segment {
    /// The first point of this segment.
    #[inline]
    pub fn a<'a>(&'a self) -> &'a Vect {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Vect {
        &self.b
    }
}

impl MeshElement for Segment {
    #[inline]
    fn nvertices(_: Option<Segment>) -> uint {
        2
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Vect], is: &[uint]) -> Segment {
        assert!(is.len() == 2);

        Segment::new(vs[is[0]].clone(), vs[is[1]].clone())
    }
}
