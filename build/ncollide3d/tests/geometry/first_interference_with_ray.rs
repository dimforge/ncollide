use std::f32;

use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use ncollide3d::{
    pipeline::{CollisionGroups, CollisionWorld, GeometricQueryType},
    query::{Ray, RayCast},
    shape::{Ball, ShapeHandle},
};

#[test]
fn first_interference_with_ray() {
    let mut world = CollisionWorld::new(0.01);

    let ball = Ball::new(1.0f32);
    let groups = CollisionGroups::new();
    let query = GeometricQueryType::Contacts(0.0, 0.0);

    // obj1
    let tra = Translation3::new(1.0, 1.0, 0.0);
    let rot = UnitQuaternion::from_scaled_axis(Vector3::y() * f32::consts::PI);
    let iso1 = Isometry3::from_parts(tra, rot);

    // obj2
    let tra = Translation3::new(10.0, 11.8, 0.0);
    let rot = UnitQuaternion::from_scaled_axis(Vector3::y() * f32::consts::PI);
    let iso2 = Isometry3::from_parts(tra, rot);

    let ray = Ray::new(
        Point3::new(0.0, 1.8, 0.0),
        Vector3::new(1.0, 1.0, 0.0).normalize(),
    );

    // Ray misses first ball and hits second
    assert!(ball
        .toi_with_ray(&iso1, &ray, std::f32::MAX, true)
        .is_none());
    assert!(ball
        .toi_with_ray(&iso2, &ray, std::f32::MAX, true)
        .is_some());
    let toi_ball = ball.toi_with_ray(&iso2, &ray, std::f32::MAX, true).unwrap();

    let shape = ShapeHandle::new(ball);
    world.add(iso1, shape.clone(), groups, query, ());
    world.add(iso2, shape.clone(), groups, query, ());
    world.update();

    // Check we've 2 objects in the world
    let num_collision_objects = world
        .collision_objects()
        .into_iter()
        .collect::<Vec<_>>()
        .len();
    assert_eq!(
        num_collision_objects, 2,
        "Expected 2 collision objects, got {}",
        num_collision_objects,
    );

    // Should have 1 intersection with ray.
    let interferences = world.interferences_with_ray(&ray, std::f32::MAX, &groups);
    let num_collisions = interferences.into_iter().collect::<Vec<_>>().len();
    assert_eq!(
        num_collisions, 1,
        "Expected 1 collision, got {}",
        num_collisions,
    );

    let first_interference = world.first_interference_with_ray(&ray, std::f32::MAX, &groups);
    let toi = first_interference.unwrap().inter.toi;

    // toi should be the same as first ball.
    assert!(
        (toi - toi_ball).abs() < 0.0001,
        "Ray collision with unexpected object",
    );
}
