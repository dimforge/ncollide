use std::num::Zero;
use nalgebra::na::Dim;
use nalgebra::na;
use geom::Triangle;
use volumetric::Volumetric;

impl<N: Zero, V: Zero + Dim, II: Zero> Volumetric<N, V, II> for Triangle<N, V> {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        assert!(na::dim::<V>() > 2, "Volumetric implementation for 2d triangles is not implemented yet.");
        (na::zero(), na::zero(), na::zero())
    }
}
