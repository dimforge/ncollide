//! Definition of the segment shape.

use std::mem;
use approx::ApproxEq;
use na::{self, Point2, Real, Unit};
use shape::{BaseMeshElement, SupportMap};
use math::{Isometry, Point};

/// A segment shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Segment<P> {
    a: P,
    b: P,
}

/// Logical description of the location of a point on a triangle.
pub enum SegmentPointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on the segment interior.
    OnEdge([N; 2]),
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

impl<P: Point> Segment<P> {
    /// The direction of this segment scaled by its length.
    ///
    /// Points from `self.a()` toward `self.b()`.
    pub fn scaled_direction(&self) -> P::Vector {
        self.b - self.a
    }

    /// The unit direction of this segment.
    ///
    /// Points from `self.a()` toward `self.b()`.
    /// Returns `None` is both points are equal.
    pub fn direction(&self) -> Option<Unit<P::Vector>> {
        Unit::try_new(self.scaled_direction(), P::Real::default_epsilon())
    }

    /// Applies the isometry `m` to the vertices of this segment and returns the resulting segment.
    pub fn transformed<M: Isometry<P>>(&self, m: &M) -> Self {
        Segment::new(m.transform_point(&self.a), m.transform_point(&self.b))
    }

    /// Computes the point at the given location.
    pub fn point_at(&self, location: &SegmentPointLocation<P::Real>) -> P {
        match *location {
            SegmentPointLocation::OnVertex(0) => self.a,
            SegmentPointLocation::OnVertex(1) => self.b,
            SegmentPointLocation::OnEdge(bcoords) => {
                let mut res = self.a;
                res.axpy(bcoords[1], &self.b, bcoords[0]);
                res
            }
            _ => unreachable!(),
        }
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
