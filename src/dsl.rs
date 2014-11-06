extern crate ncollide; // The version of ncollide does not matter here.

use ncollide::utils::symbolic::{UnivariateFn, BivariateFn, u, v, t};

fn main() {
    /*
     * Declare the symbolic variables.
     */
    let u = u();
    let v = v();
    let t = t();

    /*
     * Declare the functions.
     */
    let f1 = t * t + t * 2.0f32;
    let f2 = u * v + u * 2.0f32;

    /*
     * Evaluate them and some of their derivatives.
     */
    assert!(f1.ueval(1.0f32) == 3.0);
    assert!(f1.d1(1.0f32)    == 4.0);
    assert!(f1.d2(1.0f32)    == 2.0);
    assert!(f1.dn(1.0f32, 3) == 0.0);

    assert!(f2.beval(1.0f32, 2.0) == 4.0);
    assert!(f2.du(1.0f32, 2.0) == 4.0);
    assert!(f2.dv(1.0f32, 2.0) == 1.0);
    assert!(f2.duu(1.0f32, 2.0) == 0.0);
    assert!(f2.dvv(1.0f32, 2.0) == 0.0);
    assert!(f2.duv(1.0f32, 2.0) == 1.0);
    assert!(f2.duv_nk(1.0f32, 2.0, 3, 0) == 0.0);
}
