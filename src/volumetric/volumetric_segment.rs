use nalgebra::na;
use geom::Segment;
use volumetric::Volumetric;
use math::{N, V, II};

impl Volumetric for Segment {
    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (na::zero(), na::zero(), na::zero())
    }
}
