use na::{self, Isometry2, Point2, Vector2};
use ncollide2d::query::{Ray, RayCast};
use ncollide2d::shape::Segment;

#[test]
fn issue_178_parallel_raycast() {
    let m1 = Isometry2::identity();
    let ray = Ray::new(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(2.0, 1.0), Point2::new(2.0, 0.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    assert!(cast.is_none());
}

#[test]
fn parallel_raycast() {
    let m1 = Isometry2::identity();
    let ray = Ray::new(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(2.0, 1.0), Point2::new(2.0, -1.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    assert!(cast.is_none());
}

#[test]
fn collinear_raycast_starting_on_segment() {
    let m1 = Isometry2::identity();
    let ray = Ray::new(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(0.0, 1.0), Point2::new(0.0, -1.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    assert_eq!(cast, Some(0.0));
}

#[test]
fn collinear_raycast_starting_bellow_segment() {
    let m1 = Isometry2::identity();
    let ray = Ray::new(Point2::new(0.0, -2.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(0.0, 1.0), Point2::new(0.0, -1.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    assert_eq!(cast, Some(1.0));
}

#[test]
fn collinear_raycast_starting_above_segment() {
    let m1 = Isometry2::identity();
    let ray = Ray::new(Point2::new(0.0, 2.0), Vector2::new(0.0, 1.0));
    let seg = Segment::new(Point2::new(0.0, 1.0), Point2::new(0.0, -1.0));

    let cast = seg.toi_with_ray(&m1, &ray, true);
    assert_eq!(cast, None);
}

#[test]
fn perpendicular_raycast_starting_behind_sement() {
    let segment = Segment::new(Point2::new(0.0f32, -10.0), Point2::new(0.0, 10.0));
    let ray = Ray::new(Point2::new(-1.0, 0.0), Vector2::new(1.0, 0.0));
    assert!(segment.intersects_ray(&na::one(), &ray));
}

#[test]
fn perpendicular_raycast_starting_in_front_of_sement() {
    let segment = Segment::new(Point2::new(0.0f32, -10.0), Point2::new(0.0, 10.0));
    let ray = Ray::new(Point2::new(1.0, 0.0), Vector2::new(1.0, 0.0));
    assert!(!segment.intersects_ray(&na::one(), &ray));
}

#[test]
fn perpendicular_raycast_starting_on_segment() {
    let segment = Segment::new(Point2::new(0.0f32, -10.0), Point2::new(0.0, 10.0));
    let ray = Ray::new(Point2::new(0.0, 3.0), Vector2::new(1.0, 0.0));

    let cast = segment.toi_with_ray(&na::one(), &ray, true);
    assert_eq!(cast, Some(0.0));
}

#[test]
fn perpendicular_raycast_starting_above_segment() {
    let segment = Segment::new(Point2::new(0.0f32, -10.0), Point2::new(0.0, 10.0));
    let ray = Ray::new(Point2::new(0.0, 11.0), Vector2::new(1.0, 0.0));
    assert!(!segment.intersects_ray(&na::one(), &ray));
}

#[test]
fn perpendicular_raycast_starting_bellow_segment() {
    let segment = Segment::new(Point2::new(0.0f32, -10.0), Point2::new(0.0, 10.0));
    let ray = Ray::new(Point2::new(0.0, -11.0), Vector2::new(1.0, 0.0));
    assert!(!segment.intersects_ray(&na::one(), &ray));
}
