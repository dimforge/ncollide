use alga::general::Real;
use na::Point2;
use na;
use geometry::shape::Cuboid2;
use procedural::Polyline2;
use super::ToPolyline;
use procedural;

impl<N: Real> ToPolyline<Point2<N>, ()> for Cuboid2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let _2: N = na::convert(2.0f64);

        procedural::rectangle(&(*self.half_extents() * _2))
    }
}
