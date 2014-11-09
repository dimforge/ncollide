use na::{Pnt2, Vec2};
use shape::Triangle2;
use procedural::{ToPolyline, Polyline};
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Triangle2<N> {
    fn to_polyline(&self, _: ()) -> Polyline<N, Pnt2<N>, Vec2<N>> {
        Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
    }
}
