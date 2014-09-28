use std::num::Zero;
use na::Vec2;
use na;
use geom::Cone;
use procedural::{Polyline, ToPolyline};
use procedural;
use math::{Scalar, Vect};


#[dim2]
impl ToPolyline<()> for Cone {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        let hh = self.half_height();
        let r  = self.radius();

        let coords = vec!(Vec2::new(-r, -hh), Vec2::new(r, -hh), Vec2::new(na::zero(), hh));

        Polyline::new(coords, None)
    }
}
