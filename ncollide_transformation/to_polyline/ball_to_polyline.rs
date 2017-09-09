use alga::general::Real;
use na::Point2;
use na;
use geometry::shape::Ball2;
use procedural::Polyline2;
use procedural;
use super::ToPolyline;

impl<N: Real> ToPolyline<Point2<N>, u32> for Ball2<N> {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline2<N> {
        let diameter = self.radius() * na::convert(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
