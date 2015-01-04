use na::{Pnt2, Vec2};
use math::Scalar;
use entities::shape::Triangle2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Triangle2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
    }
}
