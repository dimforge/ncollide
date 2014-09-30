use na::Vec2;
use na;
use geom::Plane;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Vect};
use na::Indexable;

#[cfg(feature = "2d")]
impl ToPolyline<()> for Plane {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        let _0_5: Scalar = na::cast(0.5f64);
        let m0_5         = -_0_5;

        let mut res = Polyline::new(vec!(Vec2::new(m0_5, na::zero()), Vec2::new(_0_5, na::zero())), None);

        // `res` lies on the (0, x, y) plane. We have to align it with the plane normal.
        let mut axis = na::zero::<Vect>();
        axis.set(2, na::one());

        let daxis = na::cross(&axis, &self.normal());

        res.rotate_by(&daxis);

        res
    }
}
