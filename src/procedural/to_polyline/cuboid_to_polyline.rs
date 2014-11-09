use na::{Pnt2, Vec2};
use na;
use shape::Cuboid2;
use procedural::{ToPolyline, Polyline};
use procedural;
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, ()> for Cuboid2<N> {
    fn to_polyline(&self, _: ()) -> Polyline<N, Pnt2<N>, Vec2<N>> {
        let _2: N = na::cast(2.0f64);

        procedural::rectangle(&(*self.half_extents() * _2))
    }
}
