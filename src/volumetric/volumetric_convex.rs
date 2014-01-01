use geom::Convex;
use volumetric::Volumetric;
use math::{N, V, II};

impl Volumetric for Convex {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("Not yet implemented.")
    }
}
