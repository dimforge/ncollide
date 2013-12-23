use nalgebra::na;
use nalgebra::na::{Vec1, Vec2, Vec3, Mat1, Iso2, Mat3, Iso3};
use nalgebra::structs::mat::Mat3MulRhs;

pub trait InertiaTensor<N, V, AV, M> {
    fn apply(&self, a: &AV) -> AV;
    fn to_world_space(&self, &M) -> Self;
    fn to_relative_wrt_point(&self, &N, &V) -> Self;
}

pub trait Volumetric<N, V, II> {
    fn mass_properties(&self, &N) -> (N, V, II);
}

/// NOTE: it is a bit unfortunate to have to specialize that for the raw types.
impl<N: Clone + Num + Algebraic>
InertiaTensor<N, Vec2<N>, Vec1<N>, Iso2<N>> for Mat1<N> {
    #[inline]
    fn apply(&self, av: &Vec1<N>) -> Vec1<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, _: &Iso2<N>) -> Mat1<N> {
        self.clone()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &N, pt: &Vec2<N>) -> Mat1<N> {
        *self + Mat1::new(mass * na::sqnorm(pt))
    }
}

/// NOTE: it is a bit unfortunate to have to specialize that for the raw types.
impl<N: Num + Algebraic + Clone + Mat3MulRhs<N, Mat3<N>>>
InertiaTensor<N, Vec3<N>, Vec3<N>, Iso3<N>> for Mat3<N> {
    #[inline]
    fn apply(&self, av: &Vec3<N>) -> Vec3<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, t: &Iso3<N>) -> Mat3<N> {
        let inv = na::inv(&t.rotation).unwrap();
        *t.rotation.submat() * *self * *inv.submat()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &N, pt: &Vec3<N>) -> Mat3<N> {
        let diag  = na::sqnorm(pt);
        let diagm = Mat3::new(
            diag.clone(), na::zero(),   na::zero(),
            na::zero(),   diag.clone(), na::zero(),
            na::zero(),   na::zero(),   diag
        );

        *self + (diagm - na::outer(pt, pt)) * *mass
    }
}
