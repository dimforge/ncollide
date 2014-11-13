use std::num::Zero;
use na::Cross;
use na;
use volumetric::{Volumetric, InertiaTensor};
use shape::{Compound, CompoundData};
use math::{Scalar, Point, Vect, HasInertiaMatrix};


impl<N, P, V, AV, M, I> Volumetric<N, P, I> for CompoundData<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Cross<AV>, // FIXME: we are using `Cross` only to determine AV.
          M: Mul<P, P>,
          I: Zero + Add<I, I> + Mul<N, I> + InertiaTensor<N, P, AV, M> {
    fn surface(&self) -> N {
        let mut stot: N = na::zero();

        let shapes = self.shapes();
        let props = self.mass_properties_list();

        for (_, &(ref spart, _, _, _)) in shapes.iter().zip(props.iter()) {
            stot = stot + *spart;
        }

        stot
    }

    fn volume(&self) -> N {
        let mut mtot: N = na::zero();

        let shapes = self.shapes();
        let props = self.mass_properties_list();

        for (_, &(_, ref mpart, _, _)) in shapes.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
        }

        mtot
    }

    fn center_of_mass(&self) -> P {
        let mut mtot = na::zero::<N>();
        let mut ctot = na::orig::<P>();
        let mut gtot = na::orig::<P>(); // geometric center.

        let shapes = self.shapes();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, _)) in shapes.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + (*m * *cpart * *mpart).to_vec();
            gtot = gtot + (*m * *cpart).to_vec();
        }

        if mtot.is_zero() {
            gtot
        }
        else {
            ctot / mtot
        }
    }

    fn unit_angular_inertia(&self) -> I {
        let mut itot = na::zero::<I>();

        let com   = self.center_of_mass();
        let shapes = self.shapes();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, ref ipart)) in shapes.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(*mpart, &(*m * *cpart + (-*com.as_vec())));
        }

        itot
    }

    /// The mass properties of this `CompoundData`.
    ///
    /// If `density` is not zero, it will be multiplied with the density of every object of the
    /// compound shape.
    fn mass_properties(&self, density: N) -> (N, P, I) {
        let mut mtot = na::zero::<N>();
        let mut itot = na::zero::<I>();
        let mut ctot = na::orig::<P>();
        let mut gtot = na::orig::<P>(); // geometric center.

        let shapes = self.shapes();
        let props = self.mass_properties_list();

        for (&(ref m, _), &(_, ref mpart, ref cpart, _)) in shapes.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + (*m * *cpart * *mpart).to_vec();
            gtot = gtot + (*m * *cpart).to_vec();
        }

        if mtot.is_zero() {
            ctot = gtot;
        }
        else {
            ctot = ctot / mtot;
        }

        for (&(ref m, _), &(_, ref mpart, ref cpart, ref ipart)) in shapes.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(*mpart, &(*m * *cpart + (-*ctot.as_vec())));
        }

        (mtot * density, ctot, itot * density)
    }
}

impl<N, P, V, M, I> Volumetric<N, P, I> for Compound<N, P, V, M>
    where N: Scalar,
          P: Clone,
          V: HasInertiaMatrix<I>,
          I: 'static + Mul<N, I> + Clone {
    fn surface(&self) -> N {
        self.surface()
    }

    fn volume(&self) -> N {
        self.mass()
    }

    fn center_of_mass(&self) -> P {
        self.center_of_mass().clone()
    }

    fn unit_angular_inertia(&self) -> I {
        self.angular_inertia().clone()
    }
}
