//! Definition of the segment geometry.

use nalgebra::na;
use geom::mesh::MeshElement;
use math::{Scalar, Vector};

/// A segment geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Segment {
    priv margin: Scalar,
    priv a:      Vector,
    priv b:      Vector
}

impl Segment {
    /// Creates a new segment from two points.
    ///
    /// The segment will have a default margin of 0.04.
    #[inline]
    pub fn new(a: Vector, b: Vector) -> Segment {
        Segment::new_with_margin(a, b, na::cast(0.04))
    }

    /// Creates a new segment from two points and a custom margin.
    #[inline]
    pub fn new_with_margin(a: Vector, b: Vector, margin: Scalar) -> Segment {
        assert!(na::dim::<Vector>() > 1);

        Segment {
            margin: margin,
            a:      a,
            b:      b
        }
    }
}

impl Segment {
    /// The first point of this segment.
    #[inline]
    pub fn a<'a>(&'a self) -> &'a Vector {
        &'a self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a Vector {
        &'a self.b
    }

    /// The margin surrounding this segment.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}

impl MeshElement for Segment {
    #[inline]
    fn nvertices(_: Option<Segment>) -> uint {
        2
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[Vector], is: &[uint], margin: Scalar) -> Segment {
        assert!(is.len() == 2);

        Segment::new_with_margin(vs[is[0]].clone(), vs[is[1]].clone(), margin)
    }
}
