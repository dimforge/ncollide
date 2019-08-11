use na::{zero, Isometry3, Point3, Vector3};
use ncollide3d::bounding_volume::*;
use ncollide3d::shape::*;
use ncollide3d::pipeline::{CollisionGroups, CollisionWorld, GeometricQueryType};

// Issue #182.
#[test]
fn just_touching_nan3d() {
    let mut world = CollisionWorld::new(0.02f32);
    let groups = CollisionGroups::new();
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);

    let min = Point3::new(-0.5, -0.5, -0.1);
    let max = Point3::new(0.5, 0.5, 0.1);
    let bounding = AABB::new(min, max);
    let cuboid = Cuboid::new(bounding.half_extents());
    let shape = ShapeHandle::new_shared(cuboid);

    let iso1 = Isometry3::new(Vector3::zeros(), zero());
    world.add(iso1, shape.clone(), groups, contacts_query, ());

    let iso2 = Isometry3::new(Vector3::y(), zero());
    world.add(iso2, shape, groups, contacts_query, ());

    world.update();
}
