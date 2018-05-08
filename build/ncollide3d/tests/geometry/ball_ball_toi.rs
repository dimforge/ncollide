// // Issue #35

// use na::{Isometry3, Vector3};
// use ncollide::shape::Ball;
// use ncollide::query;

// #[test]
// fn test_ball_ball_toi() {
//     let b = Ball::new(0.5f64);
//     let m1 = Isometry3::new(na::zero(), na::zero());
//     let m2 = Isometry3::new(Vector3::new(0.0, 10.0, 0.0), na::zero());

//     let cast = query::time_of_impact(&m1, &Vector3::new(0.0, 10.0, 0.0), &b, &m2, &na::zero(), &b);

//     assert_eq!(cast.unwrap(), 0.9);
// }
