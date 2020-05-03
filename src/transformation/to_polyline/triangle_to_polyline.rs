use super::ToPolyline;
use crate::procedural::Polyline;
use crate::shape::Triangle;
use simba::scalar::RealField;

impl<N: RealField> ToPolyline<N> for Triangle<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        Polyline::new(vec![*self.a(), *self.b(), *self.c()], None)
    }
}
