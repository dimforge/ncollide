use super::ToPolyline;
use alga::general::RealField;
use crate::procedural::Polyline;
use crate::shape::Segment;

impl<N: RealField> ToPolyline<N> for Segment<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        Polyline::new(vec![*self.a(), *self.b()], None)
    }
}
