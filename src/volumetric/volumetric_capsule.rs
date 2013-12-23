use std::num::{Zero, One, Real};
use nalgebra::na::{Cast, Dim, Indexable};
use geom::Capsule;
use volumetric::{Volumetric, cylinder_volume, ball_volume};

#[inline]
pub fn capsule_volume<N: Zero + One + Cast<f32> + Num + Real + Clone>(
                      half_height: &N,
                      radius:      &N,
                      dim:         uint)
                      -> N {
    cylinder_volume(half_height, radius, dim) + ball_volume(radius, dim)
}

impl<N:  Zero + One + Cast<f32> + Num + Real + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Capsule<N> {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("Not yet implemented.")
    }
}
