use na::{self, Isometry2, Vector2};
use ncollide2d::narrow_phase::{ContactDispatcher, DefaultContactDispatcher};
use ncollide2d::query::contacts_internal;
use ncollide2d::query::ContactPrediction;
use ncollide2d::shape::Cuboid;
use ncollide2d::utils::IdAllocator;

#[test]
#[allow(non_snake_case)]
fn cuboid_cuboid_EPA() {
    let c = Cuboid::new(Vector2::new(2.0, 1.0));
    let m1 = Isometry2::new(Vector2::new(3.5, 0.0), na::zero());
    let m2 = Isometry2::identity();

    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, -Vector2::x_axis());

    let m1 = Isometry2::new(Vector2::new(0.0, 0.2), na::zero());
    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 1.8);
    assert_eq!(res.normal, -Vector2::y_axis());
}

#[test]
fn cuboids_large_size_ratio_issue_181() {
    let cuboid_a = Cuboid::new(Vector2::new(10.0, 10.0));
    let cuboid_b = Cuboid::new(Vector2::new(300.0, 1.5));

    let pos_b = Isometry2::new(Vector2::new(5.0, 0.0), 1.5);

    let dispatcher = DefaultContactDispatcher::new();
    let mut algorithm = dispatcher
        .get_contact_algorithm(&cuboid_a, &cuboid_b)
        .unwrap();

    let prediction = ContactPrediction::new(0.0, 0.0, 0.0);

    let mut p = Vector2::new(0.0, 0.0);
    let mut angle = 0.0;
    let mut id_alloc = IdAllocator::new();

    // Used to panic at some point:
    // thread 'main' panicked at 'assertion failed: neg_dist <= gjk::eps_tol()', ncollide_geometry/query/algorithms/EPA.rs:26:9
    for _ in 1..200000 {
        p.x += 0.0001;
        angle += 0.005;

        let pos_a = Isometry2::new(p, angle);

        algorithm.update(
            &dispatcher,
            0,
            &pos_a,
            &cuboid_a,
            0,
            &pos_b,
            &cuboid_b,
            &prediction,
            &mut id_alloc,
        );

        let mut manifolds = Vec::new();
        algorithm.contacts(&mut manifolds);

        for manifold in manifolds.iter() {
            let contact = manifold.deepest_contact().unwrap().contact.clone();
            p -= *contact.normal * contact.depth;
        }
    }
}
