// Issue #123

use na::{self, Isometry3, Point3, Vector3};
use ncollide3d::query;
use ncollide3d::shape::{Ball, Triangle};

#[test]
fn ball_triangle_toi_infinite_loop_issue() {
    let b = Ball::new(0.375f32);
    let t = Triangle::new(
        Point3::new(0.5, -0.5, 0.0),
        Point3::new(-0.5, -0.5, 0.0),
        Point3::new(-0.5, 0.5, 0.0),
    );

    let m1 = Isometry3::new(Vector3::new(0.0, 0.0, 0.0), na::zero());
    let m2 = Isometry3::new(Vector3::new(11.5, 5.5, 0.0), na::zero());
    let dir = Vector3::new(0.0, 0.000000000000000000000000000000000000000006925, 0.0);

    let cast = query::time_of_impact(
        &query::DefaultTOIDispatcher,
        &m1,
        &dir,
        &b,
        &m2,
        &na::zero(),
        &t,
        std::f32::MAX,
        0.0,
    )
    .unwrap();

    println!("TOI: {:?}", cast);
    assert!(cast.is_none()); // The provided velocity is too small.
}
