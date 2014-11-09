use na::{Pnt2, Vec2};
use na;
use shape::Cone2;
use procedural::{ToPolyline, Polyline, Polyline2};
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Cone2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let hh = self.half_height();
        let r  = self.radius();

        let coords = vec!(Pnt2::new(-r, -hh), Pnt2::new(r, -hh), Pnt2::new(na::zero(), hh));

        Polyline::new(coords, None)
    }
}
