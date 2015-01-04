use na::{Pnt2, Vec2};
use na;
use math::Scalar;
use entities::shape::Cone2;
use procedural::{Polyline, Polyline2};
use super::ToPolyline;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Cone2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let hh = self.half_height();
        let r  = self.radius();

        let coords = vec!(Pnt2::new(-r, -hh), Pnt2::new(r, -hh), Pnt2::new(na::zero(), hh));

        Polyline::new(coords, None)
    }
}
