use std::num::Float;
use na::{Pnt3, Vec3};
use na;
use shape::Torus;
use parametric::ParametricSurface;
use utils;
use math::Scalar;

/// Parametrization of the torus.
impl<N> ParametricSurface<N, Pnt3<N>, Vec3<N>> for Torus<N>
    where N: Scalar {
    fn at(&self, u: N, v: N) -> Pnt3<N> {
        let u        = u * Float::two_pi();
        let v        = v * Float::two_pi();
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.minor_radius();
        let mr       = self.major_radius();

        Pnt3::new(mr * cu, na::zero(), -mr * su) +
        Vec3::new(r * cu * cv, r * sv, -r * su * cv)
    }

    fn at_u(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = Float::two_pi();
        let u        = u * _2_pi;
        let v        = v * Float::two_pi();
        let (su, cu) = u.sin_cos();
        let cv       = v.cos();
        let r        = self.minor_radius();
        let mr       = self.major_radius();

        Vec3::new(-mr * su * _2_pi, na::zero(), -mr * cu * _2_pi) +
        Vec3::new(-r * su * cv, na::zero(), -r * cu * cv) * _2_pi
    }

    fn at_v(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = Float::two_pi();
        let u        = u * _2_pi;
        let v        = v * _2_pi;
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.minor_radius();

        Vec3::new(-r * cu * sv, r * cv, r * su * sv) * _2_pi
    }

    fn at_uu(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = Float::two_pi();
        let u        = u * _2_pi;
        let v        = v * _2_pi;
        let (su, cu) = u.sin_cos();
        let cv       = v.cos();
        let r        = self.minor_radius();
        let mr       = self.major_radius();

        Vec3::new(-mr * cu * _2_pi * _2_pi, na::zero(), mr * su * _2_pi * _2_pi) +
        Vec3::new(-r * cu * cv, na::zero(), r * su * cv) * (_2_pi * _2_pi)
    }

    fn at_vv(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = Float::two_pi();
        let u        = u * _2_pi;
        let v        = v * _2_pi;
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.minor_radius();

        Vec3::new(-r * cu * cv, -r * sv, r * su * cv) * (_2_pi * _2_pi)
    }

    fn at_uv(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = Float::two_pi();
        let u        = u * _2_pi;
        let v        = v * _2_pi;
        let (su, cu) = u.sin_cos();
        let sv       = v.sin();
        let r        = self.minor_radius();

        Vec3::new(r * su * sv, na::zero(), r * cu * sv) * (_2_pi * _2_pi)
    }

    fn at_uv_nk(&self, u: N, v: N, n: uint, k: uint) -> Vec3<N> {
        let _2_pi = Float::two_pi();
        let u  = u * _2_pi;
        let v  = v * _2_pi;
        let r  = self.minor_radius();
        let mr = self.major_radius();
        let x  =  r * utils::dcos(n, u) * utils::dcos(k, v);
        let z  = -r * utils::dsin(n, u) * utils::dcos(k, v);
        let y  = if n == 0 { r * utils::dsin(k, v) } else { na::zero() };
        let pi_n = _2_pi.powi(n as i32);

        let sphere = Vec3::new(x, y, z) * pi_n * _2_pi.powi(k as i32);

        if k == 0 {
            Vec3::new(mr * utils::dcos(n, u), na::zero(), -mr * utils::dsin(n, u)) * pi_n + sphere
        }
        else {
            sphere
        }
    }
}
