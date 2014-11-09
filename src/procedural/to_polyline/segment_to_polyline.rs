use na::{Pnt2, Vec2};
use shape::Segment2;
use procedural::{ToPolyline, Polyline};
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Segment2<N> {
    fn to_polyline(&self, _: ()) -> Polyline<N, Pnt2<N>, Vec2<N>> {
        Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
    }
}
