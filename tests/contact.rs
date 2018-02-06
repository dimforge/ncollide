extern crate ncollide;
extern crate nalgebra;

use nalgebra::{Point3, Vector3, Isometry3, zero};
use ncollide::world::{CollisionWorld, GeometricQueryType, CollisionGroups};
use ncollide::bounding_volume::*;
use ncollide::shape::*;



#[test]
fn just_touching_nan3d() {
    let mut world: CollisionWorld<Point3<f32>, Isometry3<f32>, ()> = CollisionWorld::new(0.02f32);
    let groups = CollisionGroups::new();
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);

    let min = Point3::new(-0.5, -0.5, -0.1);
    let max = Point3::new(0.5, 0.5, 0.1);
    let bounding = AABB::new(min, max);
    let cuboid = Cuboid::new(bounding.half_extents());
    let shape = ShapeHandle::new(cuboid);

    let iso1 = Isometry3::new(Vector3::zeros(), zero());
    world.add(iso1, shape.clone(), groups, contacts_query, ());

    let iso2 = Isometry3::new(Vector3::y(), zero());
    world.add(iso2, shape, groups, contacts_query, ());

    world.update();
}