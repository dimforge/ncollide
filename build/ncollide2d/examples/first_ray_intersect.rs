extern crate nalgebra as na;
extern crate ncollide2d;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::pipeline::{CollisionGroups, GeometricQueryType};
use ncollide2d::query::Ray;
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use ncollide2d::world::CollisionWorld;

fn main() {
    let ball1 = Ball::new(0.5);
    let ball2 = Ball::new(0.75);
    let cube1 = Cuboid::new(Vector2::new(0.5, 0.75));
    let cube2 = Cuboid::new(Vector2::new(1.0, 0.5));

    let shapes = [
        ShapeHandle::new(ball1),
        ShapeHandle::new(ball2),
        ShapeHandle::new(cube1),
        ShapeHandle::new(cube2),
    ];

    let poss = [
        Isometry2::new(Vector2::new(1.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(2.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(3.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(4.0, 2.0), na::zero()),
    ];

    let mut world = CollisionWorld::new(0.02);
    let collision_group = CollisionGroups::new();
    let query_type = GeometricQueryType::Contacts(0.0, 0.0);

    let z = shapes.iter().zip(&poss);
    for (shape, pos) in z {
        world.add(*pos, shape.clone(), collision_group, query_type, ());
    }

    world.update();

    let ray_hit = Ray::<f32>::new(Point2::origin(), Vector2::x());
    let ray_miss = Ray::<f32>::new(Point2::origin(), -Vector2::x());

    let hit = world
        .first_interference_with_ray(&ray_hit, &collision_group)
        .expect("Hit missed");
    let miss = world.first_interference_with_ray(&ray_miss, &collision_group);

    // println!("Hit: {:#?}", hit);
    // println!("Miss: {:#?}", miss);

    println!("Hit: {:?}", hit.inter);

    assert!(hit.inter.toi == 0.5);
    assert!(miss.is_none());

    // First object is a circle with radius 0.5 centered at (1.0, 0.0)
    // assert!(hit_toi == 0.5);
    // assert!(collector_miss.len() == 0);
}
