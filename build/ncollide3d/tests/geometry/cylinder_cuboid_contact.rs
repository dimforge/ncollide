use na::{self, Isometry3, Vector3};
use ncollide3d::query::{self, Proximity};
use ncollide3d::shape::{Cuboid, Cylinder};

// Issue #157.
#[test]
fn cylinder_cuboid_contact() {
    let cyl = Cylinder::new(0.925, 0.5);
    let cyl_at = Isometry3::new(Vector3::new(10.97, 0.925, 61.02), na::zero());
    let cuboid = Cuboid::new(Vector3::new(0.05, 0.75, 0.5));
    let cuboid_at = Isometry3::new(Vector3::new(11.50, 0.75, 60.5), na::zero());
    let distance = query::distance_support_map_support_map(
        &cyl_at, &cyl, &cuboid_at, &cuboid,
    );

    let proximity = query::proximity_support_map_support_map(
        &cyl_at, &cyl, &cuboid_at, &cuboid, 0.1,
    );

    let contact = query::contact_support_map_support_map(
        &cyl_at, &cyl, &cuboid_at, &cuboid, 10.0,
    );

    assert!(distance == 0.0);
    assert!(proximity == Proximity::Intersecting);
    assert!(contact.is_some());
}
