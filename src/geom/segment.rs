//! Definition of the segment geometry.

use nalgebra::na;
use geom::mesh::MeshElement;
use math::{N, V};

/// A segment geometry.
#[deriving(Encodable, Decodable, Clone)]
pub struct Segment {
    priv margin: N,
    priv a:      V,
    priv b:      V
}

impl Segment {
    /// Creates a new segment from two points.
    ///
    /// The segment will have a default margin of 0.04.
    #[inline]
    pub fn new(a: V, b: V) -> Segment {
        Segment::new_with_margin(a, b, na::cast(0.04))
    }

    /// Creates a new segment frow two points and a custom margin.
    #[inline]
    pub fn new_with_margin(a: V, b: V, margin: N) -> Segment {
        assert!(na::dim::<V>() > 1);

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
    pub fn a<'a>(&'a self) -> &'a V {
        &'a self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b<'a>(&'a self) -> &'a V {
        &'a self.b
    }

    /// The margin surounding this segment.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl MeshElement for Segment {
    #[inline]
    fn nvertices(_: Option<Segment>) -> uint {
        2
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[V], is: &[uint], margin: N) -> Segment {
        assert!(is.len() == 2);

        Segment::new_with_margin(vs[is[0]].clone(), vs[is[1]].clone(), margin)
    }
}
