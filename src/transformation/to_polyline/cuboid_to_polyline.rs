use alga::general::Real;
use na;
use shape::Cuboid;
use procedural::Polyline;
use super::ToPolyline;
use procedural;

impl<N: Real> ToPolyline<N> for Cuboid<N> {
    type DiscretizationParameter = ();

    fn to_polyline(&self, _: ()) -> Polyline<N> {
        let _2: N = na::convert(2.0f64);

        procedural::rectangle(&(*self.half_extents() * _2))
    }
}
