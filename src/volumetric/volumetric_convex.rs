use std::num::{Zero, One, Real};
use nalgebra::na::{Cast, Dim, Indexable};
use geom::Convex;
use volumetric::Volumetric;



impl<N:  Zero + One + Cast<f32> + Num + Real + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Convex<N, V> {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("Not yet implemented.")
    }
}
