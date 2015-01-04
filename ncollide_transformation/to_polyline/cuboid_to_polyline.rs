use na::{Pnt2, Vec2};
use na;
use math::Scalar;
use entities::shape::Cuboid2;
use procedural::Polyline2;
use super::ToPolyline;
use procedural;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Cuboid2<N> {
    fn to_polyline(&self, _: ()) -> Polyline2<N> {
        let _2: N = na::cast(2.0f64);

        procedural::rectangle(&(*self.half_extents() * _2))
    }
}
