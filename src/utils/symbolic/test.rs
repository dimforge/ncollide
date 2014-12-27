use std::rand;
use na::BaseFloat;
use na;
use utils::symbolic::{UnivariateFn, BivariateFn, U, V, T, sin, cos, exp};

#[test]
fn test_u_v() {
    assert!((U * 2.0f32 + V).beval(0.7f32, 0.6) == 2.0);
    assert!((U * 2.0f32 + V).du(0.0f32, 0.0) == 2.0);
    assert!((U * 2.0f32 + V).dv(0.0f32, 0.0) == 1.0);
    assert!((U * 2.0f32 + V).duv_nk(0.0f32, 0.0, 10, 10) == 0.0);
}

#[test]
fn test_sin_u_v() {
    let pi_2: f32 = BaseFloat::frac_pi_2();

    assert!((sin(U) + V).beval(pi_2, 0.6) == 1.6);
    assert!(na::approx_eq(&(sin(U) + V).du(pi_2, 0.6), &0.0));
    assert!((sin(U) + V).dv(0.0f32, 0.6)  == 1.0);
    assert!((sin(U) + V).duv_nk(0.0f32, 0.0, 5, 5) == 0.0);
}

#[test]
fn test_sin_t() {
    let pi_2: f64 = BaseFloat::frac_pi_2();

    assert!((sin(T) + T).ueval(pi_2)    == 1.0 + pi_2);
    assert!(na::approx_eq(&(sin(T) + T).d1(pi_2), &1.0));
    assert!(na::approx_eq(&(sin(T) + T).dn(0.0f64, 10), &0.0));
}

#[test]
fn test_duv_nk() {
    // some complicated expression.
    let expr = U * 2.0f64 - sin(cos(sin(cos(V)))) * U * V * cos(U) + 10.0f64 + exp(U * V + sin(V + 4.0f64));

    for _ in range(0u, 1000) {
        let u = rand::random::<f64>() * 5.0 - 2.5;
        let v = rand::random::<f64>() * 5.0 - 2.5;

        assert!(na::approx_eq(&expr.beval(u, v), &expr.duv_nk(u, v, 0, 0)));
        assert!(na::approx_eq(&expr.du(u, v),    &expr.duv_nk(u, v, 1, 0)));
        assert!(na::approx_eq(&expr.dv(u, v),    &expr.duv_nk(u, v, 0, 1)));
        assert!(na::approx_eq(&expr.duu(u, v),   &expr.duv_nk(u, v, 2, 0)));
        assert!(na::approx_eq(&expr.dvv(u, v),   &expr.duv_nk(u, v, 0, 2)));
        assert!(na::approx_eq(&expr.duv(u, v),   &expr.duv_nk(u, v, 1, 1)));
    }
}

#[test]
fn test_dn() {
    // some complicated expression.
    let expr = T * 2.0f64 - sin(cos(sin(cos(T)))) * T * T * cos(T) + 10.0f64 + exp(T * T + sin(T + 4.0f64));

    for _ in range(0u, 1000) {
        let t = rand::random::<f64>() * 5.0 - 2.5;

        assert!(na::approx_eq(&expr.ueval(t), &expr.dn(t, 0)));
        assert!(na::approx_eq(&expr.d1(t),    &expr.dn(t, 1)));
        assert!(na::approx_eq(&expr.d2(t),    &expr.dn(t, 2)));
    }
}
