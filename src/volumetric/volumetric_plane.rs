use std::num::Zero;
use nalgebra::na;
use geom::Plane;
use volumetric::Volumetric;

impl<N: Zero, V: Zero, II: Zero> Volumetric<N, V, II> for Plane<N, V> {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (na::zero(), na::zero(), na::zero())
    }
}
