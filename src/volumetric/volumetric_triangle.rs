use nalgebra::na;
use geom::Triangle;
use volumetric::Volumetric;
use math::{N, V, II};

impl Volumetric for Triangle {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        assert!(na::dim::<V>() > 2, "Volumetric implementation for 2d triangles is not implemented yet.");
        (na::zero(), na::zero(), na::zero())
    }
}
