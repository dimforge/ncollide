use super::ToPolyline;
use crate::procedural::Polyline;
use crate::shape::Segment;
use simba::scalar::RealField;

impl<N: RealField> ToPolyline<N> for Segment<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        Polyline::new(vec![self.a, self.b], None)
    }
}
