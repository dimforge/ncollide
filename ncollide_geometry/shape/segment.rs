//! Definition of the segment shape.

use na::{self, Dimension, Transform, Rotate, Point2};
use shape::{SupportMap, BaseMeshElement};
use math::Point;


/// A segment shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Segment<P> {
    a: P,
    b: P
}

impl<P: Dimension> Segment<P> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: P, b: P) -> Segment<P> {
        assert!(na::dimension::<P>() > 1);

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

impl<P: Dimension + Copy> BaseMeshElement<Point2<usize>, P> for Segment<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Point2<usize>) -> Segment<P> {
        Segment::new(vs[is.x], vs[is.y])
    }
}

impl<P, M> SupportMap<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        if na::dot(self.a().as_vector(), &local_dir) > na::dot(self.b().as_vector(), &local_dir) {
            m.transform(self.a())
        }
        else {
            m.transform(self.b())
        }
    }
}
