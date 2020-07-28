// Issue #35

use na::{self, Isometry2, Vector2};
use ncollide2d::query;
use ncollide2d::shape::Ball;

#[test]
fn test_ball_ball_toi() {
    let b = Ball::new(0.5f64);
    let m1 = Isometry2::new(na::zero(), na::zero());
    let m2 = Isometry2::new(Vector2::new(0.0, 10.0), na::zero());

    let cast = query::time_of_impact(
        &query::DefaultTOIDispatcher,
        &m1,
        &Vector2::new(0.0, 10.0),
        &b,
        &m2,
        &na::zero(),
        &b,
        std::f64::MAX,
        0.0,
    )
    .unwrap();

    assert_eq!(cast.unwrap().toi, 0.9);
}
