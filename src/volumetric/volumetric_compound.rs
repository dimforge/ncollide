use std::num::Zero;
use nalgebra::na::{Translation, Vec};
use nalgebra::na;
use volumetric::{Volumetric, InertiaTensor};
use geom::Compound;

impl<N:  Zero + Num,
     LV: Zero + Vec<N>,
     AV,
     M:  Translation<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
Volumetric<N, LV, II> for Compound<N, LV, M, II> {
    fn mass_properties(&self, density: &N) -> (N, LV, II) {
        let mut mtot: N  = na::zero();
        let mut itot: II = na::zero();
        let mut ctot: LV = na::zero();

        for &(ref m, ref s) in self.shapes().iter() {
            let (mpart, cpart, ipart): (N, LV, II) = s.mass_properties(density);
            mtot = mtot + mpart;
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(&mpart, &m.translation());
            ctot = ctot + cpart * mpart;
        }

        ctot = ctot / mtot;

        (mtot, ctot, itot)
    }
}
