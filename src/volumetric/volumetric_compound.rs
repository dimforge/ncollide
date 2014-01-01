use nalgebra::na::Translation;
use nalgebra::na;
use volumetric::{Volumetric, InertiaTensor};
use geom::Compound;
use math::{N, V, II};

impl Volumetric for Compound {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mut mtot: N  = na::zero();
        let mut itot: II = na::zero();
        let mut ctot: V  = na::zero();

        for &(ref m, ref s) in self.shapes().iter() {
            let (mpart, cpart, ipart): (N, V, II) = s.mass_properties(density);
            mtot = mtot + mpart;
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(&mpart, &m.translation());
            ctot = ctot + cpart * mpart;
        }

        ctot = ctot / mtot;

        (mtot, ctot, itot)
    }
}
