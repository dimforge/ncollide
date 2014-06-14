use nalgebra::na;
use volumetric::{Volumetric, InertiaTensor};
use geom::{Compound, CompoundData};
use math::{Scalar, Vect, AngularInertia};

impl Volumetric for CompoundData {
    /// The mass properties of this `CompoundData`.
    ///
    /// If `density` is not zero, it will be multiplied with the density of every object of the
    /// compound geometry.
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let mut mtot: Scalar         = na::zero();
        let mut itot: AngularInertia = na::zero();
        let mut ctot: Vect           = na::zero();

        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(ref mpart, ref cpart, _)) in geoms.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + m * *cpart * *mpart;
        }

        ctot = ctot / mtot;

        for (&(ref m, _), &(ref mpart, ref cpart, ref ipart)) in geoms.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(mpart, &(m * *cpart - ctot));
        }

        (mtot * *density, ctot, itot * *density)
    }
}

impl Volumetric for Compound {
    /// The mass properties of this compound geometry.
    ///
    /// If `density` is not zero, it will be multiplied with the density of every object used to
    /// build this compound geometry.
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let mass            = self.mass() * *density;
        let com             = self.center_of_mass();
        let angular_inertia = self.angular_inertia() * *density;

        (mass, com.clone(), angular_inertia)
    }
}
