use na::{zero, Isometry3, Point3, Vector3};
use ncollide3d::shape::*;
use ncollide3d::pipeline::{CollisionGroups, CollisionWorld, GeometricQueryType};

// Issue #188.
#[test]
fn duplicate_trimesh_on_world() {
    let mut world = CollisionWorld::new(0.02);
    let groups = CollisionGroups::new();
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);

    let vertices = vec![
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(-1.0, -1.0, 1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(-1.0, -1.0, 1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(-1.0, -1.0, 1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(-1.0, 1.0, -1.0),
        Point3::new(-1.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(-1.0, 1.0, 1.0),
    ];

    let indices = vec![
        Point3::new(0, 1, 2),
        Point3::new(3, 4, 5),
        Point3::new(6, 7, 8),
        Point3::new(9, 10, 11),
        Point3::new(12, 13, 14),
        Point3::new(15, 16, 17),
        Point3::new(18, 19, 20),
        Point3::new(21, 22, 23),
        Point3::new(24, 25, 26),
        Point3::new(27, 28, 29),
        Point3::new(30, 31, 32),
        Point3::new(33, 34, 35),
    ];

    let mesh = TriMesh::new(vertices, indices, None);

    let iso = Isometry3::new(Vector3::new(0., 0., 0.), zero());
    let shape = ShapeHandle::new_shared(mesh.clone());
    world.add(iso, shape, groups, contacts_query, ());

    let iso = Isometry3::new(Vector3::new(0., 0., 0.), zero());
    let shape = ShapeHandle::new_shared(mesh);
    world.add(iso, shape, groups, contacts_query, ());

    world.update();
}
