use nalgebra::na::Translation;
use nalgebra::na;
use volumetric::{Volumetric, InertiaTensor};
use geom::Compound;
use math::{Scalar, Vector, AngularInertia};

impl Volumetric for Compound {
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vector, AngularInertia) {
        let mut mtot: Scalar  = na::zero();
        let mut itot: AngularInertia = na::zero();
        let mut ctot: Vector  = na::zero();

        for &(ref m, ref s) in self.shapes().iter() {
            let (mpart, cpart, ipart): (Scalar, Vector, AngularInertia) = s.mass_properties(density);
            mtot = mtot + mpart;
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(&mpart, &m.translation());
            ctot = ctot + cpart * mpart;
        }

        ctot = ctot / mtot;

        (mtot, ctot, itot)
    }
}
