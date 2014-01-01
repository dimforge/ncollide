use math::{N, V, AV, M, II};

#[cfg(dim2)]
use nalgebra::na::Mat1;
#[cfg(dim2)]
use nalgebra::na;

#[cfg(dim3)]
use nalgebra::na::Mat3;
#[cfg(dim3)]
use nalgebra::na;

pub trait InertiaTensor {
    fn apply(&self, a: &AV) -> AV;
    fn to_world_space(&self, &M) -> Self;
    fn to_relative_wrt_point(&self, &N, &V) -> Self;
}

pub trait Volumetric {
    fn mass_properties(&self, &N) -> (N, V, II);
}

#[cfg(dim2)]
impl InertiaTensor for II {
    #[inline]
    fn apply(&self, av: &AV) -> AV {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, _: &M) -> II {
        self.clone()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &N, pt: &V) -> II {
        *self + Mat1::new(mass * na::sqnorm(pt))
    }
}

#[cfg(dim3)]
impl InertiaTensor for II {
    #[inline]
    fn apply(&self, av: &AV) -> AV {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, t: &M) -> II {
        let inv = na::inv(&t.rotation).unwrap();
        *t.rotation.submat() * *self * *inv.submat()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &N, pt: &V) -> II {
        let diag  = na::sqnorm(pt);
        let diagm = Mat3::new(
            diag.clone(), na::zero(),   na::zero(),
            na::zero(),   diag.clone(), na::zero(),
            na::zero(),   na::zero(),   diag
        );

        *self + (diagm - na::outer(pt, pt)) * *mass
    }
}

#[cfg(dim4)]
impl InertiaTensor for II {
    #[inline]
    fn apply(&self, _: &AV) -> AV {
        fail!("Not yet implemented.")
    }

    #[inline]
    fn to_world_space(&self, _: &M) -> II {
        fail!("Not yet implemented.")
    }

    #[inline]
    fn to_relative_wrt_point(&self, _: &N, _: &V) -> II {
        fail!("Not yet implemented.")
    }
}
