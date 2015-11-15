//! Definition of the segment shape.

use na::{Dim, Pnt2};
use na;
use shape::BaseMeshElement;


/// A segment shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
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

impl<P: Dim + Copy> BaseMeshElement<Pnt2<u32>, P> for Segment<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Pnt2<u32>) -> Segment<P> {
        Segment::new(vs[is.x as usize], vs[is.y as usize])
    }
}
