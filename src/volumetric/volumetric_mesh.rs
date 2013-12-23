use std::num::Zero;
use nalgebra::na;
use geom::Mesh;
use volumetric::Volumetric;

impl<N: Zero, V: Zero, M, II: Zero, E> Volumetric<N, V, II> for Mesh<N, V, M, II, E> {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (na::zero(), na::zero(), na::zero())
    }
}
