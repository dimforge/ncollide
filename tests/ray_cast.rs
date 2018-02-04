// extern crate nalgebra as na;
// extern crate ncollide;

// use na::{Isometry2, Point2, Vector2};
// use ncollide::shape::Segment;
// use ncollide::query::{self, RayCast};

// #[test]
// fn issue_178_parallel_raycast() {
//     let m1 = Isometry2::identity();
//     let ray = query::Ray::new(Point2::new(3.0, 0.0), Vector2::new(0.0, 1.0));
//     let seg = Segment::new(Point2::new(2.0, 1.0), Point2::new(2.0, 0.0));

//     let cast = seg.toi_with_ray(&m1, &ray, true);
//     println!("{:?}", cast);
//     assert!(cast.is_none());
// }
