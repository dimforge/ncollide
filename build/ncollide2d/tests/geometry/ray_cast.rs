use na::{self, Isometry2, Point2, Vector2};
use ncollide2d::query::{Ray, RayCast};
use ncollide2d::shape::{ConvexPolygon, Segment, Shape};

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

///    Ray Target
///    +
/// 3  |     Ray moved up each iteration
/// |  v     ^
/// 2  *--+  |
/// |  |  |  +
/// 1  +--+  *<----+Ray origin
/// |
/// 0--1--2--3
///
/// Tests the accuracy of raycaster collision detection against a `ConvexPolygon`.
#[test]
fn convexpoly_raycast_fuzz() {
    let vertices: Vec<Point2<f64>> = vec![[1, 1], [2, 1], [2, 2], [1, 2]]
        .into_iter()
        .map(|[x, y]| Point2::new(x as f64, y as f64))
        .collect();
    let square = ConvexPolygon::try_new(vertices).unwrap();
    let raycaster = square.as_ray_cast().unwrap();
    let test_raycast = |ray_origin: Point2<f64>, ray_look_at: Point2<f64>| -> Option<f64> {
        let ray_angle: Vector2<f64> = ray_look_at - ray_origin;
        raycaster.toi_with_ray(
            &Isometry2::identity(),
            &Ray::new(ray_origin, ray_angle.normalize()),
            true,
        )
    };
    for i in 8..10_000 {
        let ray_origin = Point2::new(3., 1. + (i as f64 * 0.0001));
        let ray_look_at = Point2::new(0., 2.);
        let collision = test_raycast(ray_origin, ray_look_at);
        match collision {
            Some(distance) if distance >= 1.0 && distance < (2.0f64).sqrt() => (),
            Some(distance) if distance >= 2.0 => panic!(
                "Collided with back face instead of front face. Distance: {}",
                distance
            ),
            Some(distance) => panic!("Invalid collision distance: {}", distance),
            None => panic!(
                "Failed to collide with any face: {}, {}, {}",
                i, ray_origin, ray_look_at
            ),
        }
    }
}
