//! Definition of the segment shape.

use na::Dim;
use na;
use shape::MeshElement;


/// A segment shape.
#[derive(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
pub struct Segment<P> {
    a: P,
    b: P
}

impl<P: Dim> Segment<P> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: P, b: P) -> Segment<P> {
        assert!(na::dim::<P>() > 1);

        Segment {
            a: a,
            b: b
        }
    }
}

impl<P> Segment<P> {
    /// The first point of this segment.
    #[inline]
    pub fn a(&self) -> &P {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b(&self) -> &P {
        &self.b
    }
}

impl<P: Dim + Clone> MeshElement<P> for Segment<P> {
    #[inline]
    fn nvertices(_: Option<Segment<P>>) -> uint {
        2
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &[uint]) -> Segment<P> {
        assert!(is.len() == 2);

        Segment::new(vs[is[0]].clone(), vs[is[1]].clone())
    }
}
