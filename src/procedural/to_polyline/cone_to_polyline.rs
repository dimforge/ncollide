use na::Pnt2;
use na;
use geom::Cone;
use procedural::{Polyline, ToPolyline};
use math::{Scalar, Point, Vect};


#[cfg(feature = "2d")]
impl ToPolyline<()> for Cone {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Point, Vect> {
        let hh = self.half_height();
        let r  = self.radius();

        let coords = vec!(Pnt2::new(-r, -hh), Pnt2::new(r, -hh), Pnt2::new(na::zero(), hh));

        Polyline::new(coords, None)
    }
}
