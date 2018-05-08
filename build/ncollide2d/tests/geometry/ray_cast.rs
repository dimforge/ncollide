use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::Segment;
use ncollide2d::query::{self, RayCast};

#[test]
fn issue_178_parallel_raycast() {
    let m1 = Isometry2::identity();
    let ray = query::Ray::new(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(2.0, 1.0), Point2::new(2.0, 0.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    println!("{:?}", cast);
    assert!(cast.is_none());
}

#[test]
fn parallel_raycast() {
    let m1 = Isometry2::identity();
    let ray = query::Ray::new(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(2.0, 1.0), Point2::new(2.0, -1.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    println!("{:?}", cast);
    assert!(cast.is_none());
}
