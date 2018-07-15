use na::{self, Isometry3, Vector3};
use ncollide3d::query::contacts_internal;
use ncollide3d::shape::Cuboid;

#[test]
#[allow(non_snake_case)]
fn cuboid_cuboid_EPA() {
    let c = Cuboid::new(Vector3::new(2.0, 1.0, 1.0));
    let m12 = Isometry3::new(Vector3::new(3.5, 0.0, 0.0), na::zero());

    let res = contacts_internal::support_map_against_support_map(&c, &m12, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, -Vector3::x_axis());

    let m11 = Isometry3::new(Vector3::new(3.5, -0.2, 0.0), na::zero());
    let res = contacts_internal::support_map_against_support_map(&c, &m12, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 1.8);
    assert_eq!(res.normal, -Vector3::y_axis());
}
