use std::num::Zero;
use na;
use volumetric::{Volumetric, InertiaTensor};
use geom::{Compound, CompoundData};
use math::{Scalar, Point, AngularInertia};

impl Volumetric for CompoundData {
    fn surface(&self) -> Scalar {
        let mut stot: Scalar = na::zero();

        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (_, &(ref spart, _, _, _)) in geoms.iter().zip(props.iter()) {
            stot = stot + *spart;
        }

        stot
    }

    fn volume(&self) -> Scalar {
        let mut mtot: Scalar = na::zero();

        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (_, &(_, ref mpart, _, _)) in geoms.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
        }

        mtot
    }

    fn center_of_mass(&self) -> Point {
        let mut mtot: Scalar = na::zero();
        let mut ctot: Point  = na::orig();
        let mut gtot: Point  = na::orig(); // geometric center.

        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, _)) in geoms.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + (m * *cpart * *mpart).to_vec();
            gtot = gtot + (m * *cpart).to_vec();
        }

        if mtot.is_zero() {
            gtot
        }
        else {
            ctot / mtot
        }
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let mut itot: AngularInertia = na::zero();

        let com   = self.center_of_mass();
        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, ref ipart)) in geoms.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(mpart, &(m * *cpart - *com.as_vec()));
        }

        itot
    }

    /// The mass properties of this `CompoundData`.
    ///
    /// If `density` is not zero, it will be multiplied with the density of every object of the
    /// compound geometry.
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Point, AngularInertia) {
        let mut mtot: Scalar         = na::zero();
        let mut itot: AngularInertia = na::zero();
        let mut ctot: Point          = na::orig();
        let mut gtot: Point          = na::orig(); // egometric center.

        let geoms = self.geoms();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, _)) in geoms.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + (m * *cpart * *mpart).to_vec();
            gtot = gtot + (m * *cpart).to_vec();
        }

        if mtot.is_zero() {
            ctot = gtot;
        }
        else {
            ctot = ctot / mtot;
        }

        for (&(ref m, _), &(_, ref mpart, ref cpart, ref ipart)) in geoms.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(mpart, &(m * *cpart - *ctot.as_vec()));
        }

        (mtot * *density, ctot, itot * *density)
    }
}

impl Volumetric for Compound {
    fn surface(&self) -> Scalar {
        self.surface()
    }

    fn volume(&self) -> Scalar {
        self.mass()
    }

    fn center_of_mass(&self) -> Point {
        self.center_of_mass().clone()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        self.angular_inertia().clone()
    }
}
