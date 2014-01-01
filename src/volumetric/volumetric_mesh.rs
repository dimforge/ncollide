use nalgebra::na;
use geom::Mesh;
use volumetric::Volumetric;
use math::{N, V, II};

impl Volumetric for Mesh {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (na::zero(), na::zero(), na::zero())
    }
}
