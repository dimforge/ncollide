use geom::Capsule;
use volumetric::{Volumetric, cylinder_volume, ball_volume};
use math::{N, V, II};

#[inline]
pub fn capsule_volume(half_height: &N, radius: &N, dim: uint) -> N {
    cylinder_volume(half_height, radius, dim) + ball_volume(radius, dim)
}

impl Volumetric for Capsule {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("Not yet implemented.")
    }
}
