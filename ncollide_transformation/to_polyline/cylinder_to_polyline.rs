use na::{Point2, Vector2};
use na;
use math::Scalar;
use entities::shape::Cylinder2;
use procedural::Polyline2;
use procedural;
use super::ToPolyline;

impl<N: Scalar> ToPolyline<Point2<N>, ()> for Cylinder2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let _2: N = na::cast(2.0f64);

        procedural::rectangle(&Vector2::new(self.radius() * _2, self.half_height() * _2))
    }
}
