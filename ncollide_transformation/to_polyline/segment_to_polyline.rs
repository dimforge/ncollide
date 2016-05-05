use na::Point2;
use math::Scalar;
use geometry::shape::Segment2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Scalar> ToPolyline<Point2<N>, ()> for Segment2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
    }
}
