use std::f32;

use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use ncollide3d::{
    query::{Ray, RayCast},
    shape::{Ball, ShapeHandle},
    world::{CollisionGroups, CollisionWorld, GeometricQueryType},
};

#[test]
fn interferences_with_ray() {
    let mut world = CollisionWorld::new(0.01);

    let ball = Ball::new(0.5f32);
    let groups = CollisionGroups::new();
    let query = GeometricQueryType::Contacts(0.0, 0.0);

    let tra = Translation3::new(1.0, 1.0, 1.0);
    let rot = UnitQuaternion::from_scaled_axis(Vector3::y() * f32::consts::PI);
    let iso = Isometry3::from_parts(tra, rot);

    let ray = Ray::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

    assert!(ball.toi_with_ray(&iso, &ray, true,).is_some());

    let shape = ShapeHandle::new(ball);
    world.add(iso, shape.clone(), groups, query, ());
    world.update();

    let num_collision_objects = world
        .collision_objects()
        .into_iter()
        .collect::<Vec<_>>()
        .len();
    assert!(
        num_collision_objects == 1,
        format!("Expected 1 collision object, got {}", num_collision_objects),
    );

    let interferences = world.interferences_with_ray(&ray, &groups);
    let num_collisions = interferences.into_iter().collect::<Vec<_>>().len();
    assert!(
        num_collisions == 1,
        format!("Expected 1 collision, got {}", num_collisions),
    );
}
