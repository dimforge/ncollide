extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry3, Vector3};
use na::{Isometry2, Vector2};
use ncollide::shape::Cuboid;
use ncollide::query::contacts_internal;

#[test]
fn cuboid_cuboid_epa3() {
    let c   = Cuboid::new(Vector3::new(2.0, 1.0, 1.0));
    let m1  = Isometry3::new(Vector3::new(3.5, 0.0, 0.0), na::zero());
    let m2  = Isometry3::identity();

    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
              .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, Vector3::x());
}

#[test]
fn cuboid_cuboid_epa2() {
    let c   = Cuboid::new(Vector2::new(2.0, 1.0));
    let m1  = Isometry2::new(Vector2::new(3.5, 0.0), na::zero());
    let m2  = Isometry2::identity();

    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
              .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, -Vector2::x());

    let m1  = Isometry2::new(Vector2::new(0.0, 0.2), na::zero());
    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
              .expect("Penetration not found.");
    assert_eq!(res.depth, 1.8);
    assert_eq!(res.normal, -Vector2::y());
}
