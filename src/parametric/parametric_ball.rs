#![allow(non_snake_case)] // For alignment sake.

use na::{Pnt3, Vec3, BaseFloat};
use na;
use shape::Ball;
use parametric::ParametricSurface;
use utils;
use math::Scalar;

/// Parametrization of the ball and its derivatives:
///
/// S(u, v)    = ( r cos(u) cos(v), r sin(v) , -r sin(u) cos(v))
/// S_u(u, v)  = (-r sin(u) cos(v), 0        , -r cos(u) cos(v))
/// S_v(u, v)  = (-r cos(u) sin(v), r cos(v) ,  r sin(u) sin(v))
/// S_uu(u, v) = (-r cos(u) cos(v), 0        ,  r sin(u) cos(v))
/// S_vv(u, v) = (-r cos(u) cos(v), -r sin(v),  r sin(u) cos(v))
/// S_uv(u, v) = ( r sin(u) sin(v), 0        ,  r cos(u) sin(v))
impl<N> ParametricSurface<N, Pnt3<N>, Vec3<N>> for Ball<N>
    where N: Scalar {
    fn at(&self, u: N, v: N) -> Pnt3<N> {
        let u        = u * BaseFloat::two_pi();
        let v        = (v - na::cast(0.5f64)) * BaseFloat::pi();
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.radius();

        Pnt3::new(r * cu * cv, r * sv, -r * su * cv)
    }

    fn at_u(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = BaseFloat::two_pi();
        let u        = u * _2_pi;
        let v        = (v - na::cast(0.5f64)) * BaseFloat::pi();
        let (su, cu) = u.sin_cos();
        let cv       = v.cos();
        let r        = self.radius();

        Vec3::new(-r * su * cv, na::zero(), -r * cu * cv) * _2_pi
    }

    fn at_v(&self, u: N, v: N) -> Vec3<N> {
        let _pi      = BaseFloat::pi();
        let u        = u * BaseFloat::two_pi();
        let v        = (v - na::cast(0.5f64)) * _pi;
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.radius();

        Vec3::new(-r * cu * sv, r * cv, r * su * sv) * _pi
    }

    fn at_uu(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = BaseFloat::two_pi();
        let u        = u * _2_pi;
        let v        = (v - na::cast(0.5f64)) * BaseFloat::pi();
        let (su, cu) = u.sin_cos();
        let cv       = v.cos();
        let r        = self.radius();

        Vec3::new(-r * cu * cv, na::zero(), r * su * cv) * (_2_pi * _2_pi)
    }

    fn at_vv(&self, u: N, v: N) -> Vec3<N> {
        let _pi      = BaseFloat::pi();
        let u        = u * BaseFloat::two_pi();
        let v        = (v - na::cast(0.5f64)) * _pi;
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let r        = self.radius();

        Vec3::new(-r * cu * cv, -r * sv, r * su * cv) * (_pi * _pi)
    }

    fn at_uv(&self, u: N, v: N) -> Vec3<N> {
        let _2_pi    = BaseFloat::two_pi();
        let _pi      = BaseFloat::pi();
        let u        = u * _2_pi;
        let v        = (v - na::cast(0.5f64)) * _pi;
        let (su, cu) = u.sin_cos();
        let sv       = v.sin();
        let r        = self.radius();

        Vec3::new(r * su * sv, na::zero(), r * cu * sv) * (_pi * _2_pi)
    }

    fn at_u_v(&self, u: N, v: N) -> (Pnt3<N>, Vec3<N>, Vec3<N>) {
        let _2_pi     = BaseFloat::two_pi();
        let _pi       = BaseFloat::pi();
        let u         = u * _2_pi;
        let v         = (v - na::cast(0.5f64)) * _pi;
        let (su, cu)  = u.sin_cos();
        let (sv, cv)  = v.sin_cos();
        let r         = self.radius();
        let r___cu___cv = r * cu * cv;
        let r___su___cv = r * su * cv;

        (
            Pnt3::new( r___cu___cv, r * sv,     -r___su___cv),
            Vec3::new(-r___su___cv, na::zero(), -r___cu___cv) * _2_pi,
            Vec3::new(-r * cu * sv, r * cv,      r * su * sv) * _pi
        )
    }

    fn at_u_v_uu_vv_uv(&self, u: N, v: N)
        -> (Pnt3<N>, Vec3<N>, Vec3<N>, Vec3<N>, Vec3<N>, Vec3<N>) {
        let _2_pi       = BaseFloat::two_pi();
        let _pi         = BaseFloat::pi();
        let u           = u * _2_pi;
        let v           = (v - na::cast(0.5f64)) * _pi;
        let (su, cu)    = u.sin_cos();
        let (sv, cv)    = v.sin_cos();
        let r           = self.radius();
        let r___sv      = r * sv;
        let r_cu_cv = r * cu * cv;
        let r_su_sv = r * su * sv;
        let r_su_cv = r * su * cv;
        let r_cu_sv = r * cu * sv;

        (
            Pnt3::new( r_cu_cv, r___sv,     -r_su_cv),
            Vec3::new(-r_su_cv, na::zero(), -r_cu_cv) * _2_pi,
            Vec3::new(-r_cu_sv, r * cv,      r_su_sv) * _pi,
            Vec3::new(-r_cu_cv, na::zero(),  r_su_cv) * _2_pi * _2_pi,
            Vec3::new(-r_cu_cv, -r___sv,     r_su_cv) * _pi * _pi,
            Vec3::new( r_su_sv, na::zero(),  r_cu_sv) * _2_pi * _pi
        )
    }

    fn at_uv_nk(&self, u: N, v: N, n: uint, k: uint) -> Vec3<N> {
        let _2_pi = BaseFloat::two_pi();
        let _pi   = BaseFloat::pi();
        let u     = u * _2_pi;
        let v     = (v - na::cast(0.5f64)) * _pi;
        let r     = self.radius();
        let x     =  r * utils::dcos(n, u) * utils::dcos(k, v);
        let z     = -r * utils::dsin(n, u) * utils::dcos(k, v);
        let y     = if n == 0 { r * utils::dsin(k, v) } else { na::zero() };

        Vec3::new(x, y, z) * _2_pi.powi(n as i32) * _pi.powi(k as i32)
    }

    // FIXME: implement explicitely `at_uv_n` to prevent repeated evulation of `dcos` and `dsin`.
}
