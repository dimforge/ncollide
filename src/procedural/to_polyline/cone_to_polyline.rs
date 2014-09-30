use na::Vec2;
use na;
use geom::Cone;
use procedural::{Polyline, ToPolyline};
use math::{Scalar, Vect};


#[cfg(feature = "2d")]
impl ToPolyline<()> for Cone {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        let hh = self.half_height();
        let r  = self.radius();

        let coords = vec!(Vec2::new(-r, -hh), Vec2::new(r, -hh), Vec2::new(na::zero(), hh));

        Polyline::new(coords, None)
    }
}
