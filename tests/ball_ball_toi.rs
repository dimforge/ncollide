// Issue #35

extern crate nalgebra as na;
extern crate ncollide;

use na::{Iso3, Vec3};
use ncollide::shape::Ball;
use ncollide::geometry;

#[test]
fn test_ball_ball_toi() {
    let b  = Ball::new(0.5f64);
    let m1 = Iso3::new(na::zero(), na::zero());
    let m2 = Iso3::new(Vec3::new(0.0, 10.0, 0.0), na::zero());

    let cast = geometry::time_of_impact(&m1, &Vec3::new(0.0, 10.0, 0.0), &b, &m2, &na::zero(), &b);

    assert_eq!(cast.unwrap(), 0.9);
}
