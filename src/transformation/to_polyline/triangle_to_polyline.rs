use alga::general::Real;
use na::Point2;
use shape::Triangle;
use procedural::Polyline;
use super::ToPolyline;

impl<N: Real> ToPolyline<N> for Triangle<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        Polyline::new(vec![*self.a(), *self.b(), *self.c()], None)
    }
}
