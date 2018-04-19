use alga::general::Real;
use na::Point2;
use shape::Triangle2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Real> ToPolyline<Point2<N>, ()> for Triangle2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        Polyline::new(vec![*self.a(), *self.b(), *self.c()], None)
    }
}
