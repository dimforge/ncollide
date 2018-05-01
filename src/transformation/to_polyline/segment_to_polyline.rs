use alga::general::Real;
use shape::Segment;
use procedural::Polyline;
use super::ToPolyline;

impl<N: Real> ToPolyline<N> for Segment<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        Polyline::new(vec![*self.a(), *self.b()], None)
    }
}
