//! Definition of the segment shape.

use std::mem;
use na::{self, Point2};
use shape::{BaseMeshElement, SupportMap};
use math::{Isometry, Point};

/// A segment shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Segment<P> {
    a: P,
    b: P,
}

impl<P: Point> Segment<P> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: P, b: P) -> Segment<P> {
        assert!(na::dimension::<P::Vector>() > 1);
        Segment { a, b }
    }

    /// Creates the reference to a segment from the reference to an array of two points.
    pub fn from_array(arr: &[P; 2]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
    }

    pub(crate) fn from_array3(arr: &[P; 3]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
    }

    pub(crate) fn from_array4(arr: &[P; 4]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
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

impl<P: Point> BaseMeshElement<Point2<usize>, P> for Segment<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Point2<usize>) -> Segment<P> {
        Segment::new(vs[is.x], vs[is.y])
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Segment<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_transform_vector(dir);

        if na::dot(&self.a().coordinates(), &local_dir)
            > na::dot(&self.b().coordinates(), &local_dir)
        {
            m.transform_point(self.a())
        } else {
            m.transform_point(self.b())
        }
    }
}
