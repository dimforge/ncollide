use alga::general::Real;
use na::Point2;
use na;
use shape::Cone2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Real> ToPolyline<Point2<N>, ()> for Cone2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let hh = self.half_height();
        let r = self.radius();

        let coords = vec![
            Point2::new(-r, -hh),
            Point2::new(r, -hh),
            Point2::new(na::zero(), hh),
        ];

        Polyline::new(coords, None)
    }
}
