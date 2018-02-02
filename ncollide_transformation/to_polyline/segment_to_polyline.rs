use alga::general::Real;
use na::Point2;
use geometry::shape::Segment2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Real> ToPolyline<Point2<N>, ()> for Segment2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        Polyline::new(vec![*self.a(), *self.b()], None)
    }
}
